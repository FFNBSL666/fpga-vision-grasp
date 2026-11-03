"""Export the trained QONNX weights to a sequential, native-Verilog QNN.

This is an alternative hardware backend for Vivado 2018.3. It is NOT an IP
emitted by FINN's HLS compiler. All arithmetic/lookup semantics are recorded.
The original trained and FINN-converted graphs are kept unchanged.
"""
from pathlib import Path
from fractions import Fraction
import hashlib
import json
import math
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work'/'qonnx_compat'))


def even_round(value):
    q,r=divmod(value.numerator,value.denominator)
    return q+int(2*r>value.denominator or (2*r==value.denominator and q%2))


def conv_int(x,w,stride,pad):
    import numpy as np
    x=np.pad(x,((0,0),(pad,pad),(pad,pad)))
    k=w.shape[-1]
    windows=np.lib.stride_tricks.sliding_window_view(x,(k,k),axis=(1,2))[:,::stride,::stride]
    return np.einsum('ockl,cxykl->oxy',w.astype('int32'),windows.astype('int32'),optimize=True)


def integer_forward(packed,p):
    import numpy as np
    a=p['input_lut'][packed]
    traces=[]
    for layer in range(4):
        acc=conv_int(a,p['weights'][layer],2 if layer<2 else 1,1 if layer<3 else 0)
        if layer==3:return acc,traces
        a=np.searchsorted(p['thresholds'][layer],acc,side='right').astype('uint8')
        traces.append(a.copy())
        if layer<2:
            c,h,w=a.shape;a=a.reshape(c,h//2,2,w//2,2).max(axis=(2,4))


def decode_reference(head,p):
    import numpy as np
    result=[]
    for cid in range(2):
        offset=cid*5;iy,ix=np.unravel_index(np.argmax(head[offset]),(6,6))
        values=[]
        for c in (offset,offset+1,offset+2):
            idx=((int(head[c,iy,ix])*int(p['affine_mul'][c])+int(p['affine_bias'][c])+(1<<23))>>24)+256
            idx=max(0,min(511,idx));values.append(int(p['sigmoid_lut'][idx]))
        result.append({'valid':bool(head[offset,iy,ix]>=p['object_threshold'][cid]),
                       'confidence':values[0],'x':int(ix*16+values[1]//16),'y':int(iy*16+values[2]//16),
                       'grid_x':int(ix),'grid_y':int(iy)})
    return result


def load_params(folder):
    import numpy as np
    z=np.load(Path(folder)/'integer_params.npz')
    return {'weights':[z[f'w{i}']for i in range(4)],'thresholds':[z[f't{i}']for i in range(3)],
            **{n:z[n]for n in ['input_lut','head_scale','head_bias','affine_mul','affine_bias','object_threshold','sigmoid_lut']}}


def main():
    import numpy as np
    import onnx
    from onnx import numpy_helper
    source=ROOT/'model_candidates'/'grid_offset_v3_20261024'/'candidate_qonnx.onnx'
    model=onnx.load(str(source));init={v.name:numpy_helper.to_array(v)for v in model.graph.initializer}
    qnodes=[n for n in model.graph.node if n.op_type=='Quant']
    def scalar_quant(n):
        attrs={a.name:onnx.helper.get_attribute_value(a)for a in n.attribute}
        return float(init[n.input[1]]),attrs
    input_scale,input_attrs=scalar_quant(qnodes[0]);assert input_attrs['signed']==1
    scale_in=input_scale;weights=[];thresholds=[];weight_scales=[];activation_scales=[]
    for layer,feature in enumerate([0,3,6,8]):
        n=next(n for n in qnodes if n.input[0]==f'features.{feature}.weight')
        scale,attrs=scalar_quant(n)
        assert np.asarray(init[n.input[1]]).shape==() and attrs['signed']==1
        narrow=int(attrs.get('narrow',0));low=-128+narrow
        w=np.clip(np.rint(init[n.input[0]]/scale),low,127).astype('int8');weights.append(w);weight_scales.append(scale)
        if layer<3:
            act=next(n for n in qnodes if f'/features/features.{feature+1}/act_quant/' in n.input[1])
            scale_out,aattrs=scalar_quant(act);assert aattrs['signed']==0
            gain=Fraction(scale_in)*Fraction(scale)/Fraction(scale_out)
            t=[]
            for k in range(255):
                boundary=Fraction(2*k+1,2)/gain
                q,r=divmod(boundary.numerator,boundary.denominator)
                t.append(q+int(bool(r) or k%2==0))
            thresholds.append(np.array(t,dtype='int64'));activation_scales.append(scale_out);scale_in=scale_out
    input_lut=np.array([min(127,max(0,even_round(Fraction(u,255)/Fraction(input_scale))))for u in range(256)],dtype='uint8')
    bias=init['features.8.bias'].astype('float64');head_scale=scale_in*weight_scales[-1]
    mul=np.array([round(head_scale*32*(1<<24))]*10,dtype='int64')
    offs=np.rint(bias*32*(1<<24)).astype('int64')
    object_threshold=np.array([math.ceil(-bias[c]/head_scale)for c in [0,5]],dtype='int64')
    sigmoid=np.rint(255/(1+np.exp(-((np.arange(512)-256)/32)))).astype('uint8')
    out=ROOT/'model_candidates'/'native_qnn_v3_20261024';out.mkdir(parents=True,exist_ok=True)
    np.savez(out/'integer_params.npz',**{f'w{i}':w for i,w in enumerate(weights)},
             **{f't{i}':t for i,t in enumerate(thresholds)},input_lut=input_lut,
             head_scale=np.array(head_scale),head_bias=bias,affine_mul=mul,affine_bias=offs,
             object_threshold=object_threshold,sigmoid_lut=sigmoid)
    def mem(name,values,bits):
        (out/name).write_text(''.join(f'{int(v)&((1<<bits)-1):0{bits//4}x}\n'for v in values),encoding='ascii')
    allweights=np.concatenate([w.ravel()for w in weights]);mem('qnn_weights.mem',np.pad(allweights,(0,32768-len(allweights))),8)
    thresh=np.zeros(1024,dtype='int64')
    for i,t in enumerate(thresholds):thresh[i*256:i*256+255]=t
    mem('qnn_thresholds.mem',thresh,32);mem('qnn_input.mem',input_lut,8);mem('qnn_sigmoid.mem',sigmoid,8)
    # 10 multiply factors, 10 signed offsets, 2 confidence thresholds, padded ROM.
    affine=np.r_[mul,offs,object_threshold,np.zeros(10,dtype='int64')];mem('qnn_affine.mem',affine,64)
    report={'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'backend':'native sequential Verilog; not FINN-generated HLS IP',
            'input_shape':[1,3,96,96],'head_shape':[10,6,6],'weight_count':len(allweights),
            'weight_scales':weight_scales,'activation_scales':activation_scales,'input_scale':input_scale,
            'head_scale':head_scale,'head_bias':bias.tolist(),'object_threshold':object_threshold.tolist(),
            'integer_semantics':'signed INT8 weights; nonnegative activation integers; exact-rational RNE thresholds; INT32 MAC; 2x2 maxpool',
            'decode_semantics':'best objectness cell per class; Q24 affine to 512-entry sigmoid LUT; 0..95 floor pixel coordinates',
            'float_equivalence':'to be evaluated, not assumed bit-exact',
            'hardware_validation':'NOT_RUN','motion_enable':False}
    (out/'export_manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
