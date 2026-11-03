from pathlib import Path
import json
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work'/'qonnx_compat'))


def main():
    import numpy as np
    from PIL import Image
    from qonnx.core.modelwrapper import ModelWrapper
    from qonnx.core.onnx_exec import execute_onnx
    from qonnx.transformation.general import GiveUniqueNodeNames
    from qonnx.transformation.infer_shapes import InferShapes
    from prepare_camera_vectors import resize_reference,rgb565_roundtrip
    from export_integer_qnn import integer_forward,decode_reference,load_params
    out=ROOT/'model_candidates'/'native_qnn_v3_20261024';p=load_params(out)
    rows=[json.loads(s) for s in Path(r'F:\CODEX\preboard_mayuan6_rc1_20261021\dataset\manifest.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    rows=[r for r in rows if r['split']=='testing']
    qmodel=ModelWrapper(str(ROOT/'model_candidates'/'grid_offset_v3_20261024'/'candidate_qonnx.onnx')).transform(GiveUniqueNodeNames()).transform(InferShapes())
    changes=0;grid_changes=0;max_logit=0;max_center_delta=0;empty=empty_fp=0;start=time.monotonic()
    stats={n:{'truth':0,'predicted':0,'matched':0,'errors':[]}for n in ['finger_tip','object']}
    for i,row in enumerate(rows):
        image=Image.open(row['image']).convert('RGB').resize((160,120),Image.Resampling.BILINEAR)
        _,expanded=rgb565_roundtrip(np.asarray(image));rgb=resize_reference(expanded);chw=rgb.transpose(2,0,1)
        head,traces=integer_forward(chw,p);native=decode_reference(head,p)
        qout=next(iter(execute_onnx(qmodel,{qmodel.graph.input[0].name:chw.astype('float32')[None]/255}).values())).reshape(10,6,6)
        max_logit=max(max_logit,float(np.max(np.abs(head*p['head_scale']+p['head_bias'][:,None,None]-qout))))
        any_pred=False
        for cid,name in enumerate(stats):
            off=cid*5;iy,ix=np.unravel_index(np.argmax(qout[off]),(6,6));pred=native[cid]
            qv=1/(1+np.exp(-np.clip(qout[off:off+3,iy,ix],-80,80)))
            changes+=int(pred['valid']!=(qv[0]>=.5));grid_changes+=int((pred['grid_y'],pred['grid_x'])!=(iy,ix))
            max_center_delta=max(max_center_delta,float(np.hypot(pred['x']-(ix+qv[1])*16,pred['y']-(iy+qv[2])*16)))
            boxes=[b for b in row['boxes']if int(b['class_id'])==cid];gt=max(boxes,key=lambda b:b['w']*b['h'])if boxes else None
            s=stats[name];s['truth']+=int(gt is not None);s['predicted']+=int(pred['valid']);any_pred|=pred['valid']
            if gt and pred['valid']:
                s['matched']+=1;s['errors'].append(float(np.hypot(pred['x']-gt['cx']*96,pred['y']-gt['cy']*96)))
        if not row['boxes']:empty+=1;empty_fp+=int(any_pred)
        if True:
            case=out/f'case{i}';case.mkdir(exist_ok=True)
            pixels=(rgb[:,:,0].astype('uint32')<<16)|(rgb[:,:,1].astype('uint32')<<8)|rgb[:,:,2]
            (case/'pixels.mem').write_text(''.join(f'{int(v):06x}\n'for v in pixels.flat),encoding='ascii')
            (case/'head.mem').write_text(''.join(f'{int(v)&0xffffffff:08x}\n'for v in head.flat),encoding='ascii')
            if i<3:
                for l,t in enumerate(traces):(case/f'act{l}.mem').write_text(''.join(f'{int(v):02x}\n'for v in t.flat),encoding='ascii')
            (case/'expected.json').write_text(json.dumps({'image':row['relative_image'],'detections':native},indent=2),encoding='utf-8')
            word=0
            for cid,v in enumerate(native):word|=(int(v['valid'])|(v['confidence']<<1)|(v['x']<<9)|(v['y']<<17))<<(cid*25)
            (case/'detection.mem').write_text(f'{word:013x}\n',encoding='ascii')
            if i==0:
                import binascii
                raw,_=rgb565_roundtrip(np.asarray(image));header=bytes((2,37,160,120,1));payload=raw.astype('<u2').tobytes()
                packet=b'\xa5\x5a'+header+payload+binascii.crc_hqx(header+payload,0xffff).to_bytes(2,'big')
                (case/'camera_packet.mem').write_text(''.join(f'{v:02x}\n'for v in packet),encoding='ascii')
        if (i+1)%10==0:print(f'Integer verification {i+1}/{len(rows)}',flush=True)
    for s in stats.values():
        errors=s.pop('errors');s['mean_center_error_pixels']=float(np.mean(errors))if errors else None
    report={'images':len(rows),'confidence_decision_changes_vs_qonnx':changes,'argmax_grid_changes_vs_qonnx':grid_changes,
            'max_head_logit_error_vs_qonnx':max_logit,'max_center_delta_pixels_vs_qonnx':max_center_delta,
            'classes':stats,'empty_images':empty,'empty_false_positive_images':empty_fp,
            'status':'PASS' if changes==0 and grid_changes==0 and max_center_delta<1.5 else 'REVIEW_REQUIRED',
            'comparison_note':'Integer native backend, not float-bit-exact FINN HLS. Coordinate floor/LUT is explicitly compared.',
            'seconds':time.monotonic()-start,'rtl_validation':'NOT_RUN','real_board_validation':'NOT_RUN'}
    (ROOT/'reports'/'native_qnn_numeric_20261024.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if report['status']!='PASS':sys.exit(1)


if __name__=='__main__':main()
