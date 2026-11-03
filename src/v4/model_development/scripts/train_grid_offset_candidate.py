"""Local candidate only: correct global-XY regression into grid-relative XY.

Keeps the existing 8-bit network. Does not overwrite RC1 or select a final model.
Uses training split for optimization; testing split is reported separately.
All preprocessing emulates RGB565 plus the actual FPGA Q8 bilinear resizer.
"""
from pathlib import Path
import json
import random
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work'/'qonnx_compat'))
sys.path.insert(0,r'F:\CODEX\finn_mayuan')


def main():
    import numpy as np
    import torch
    from PIL import Image
    from train_qnn_detector_v2_96 import TinyQNNDetector
    from prepare_camera_vectors import rgb565_roundtrip,resize_reference
    from brevitas.export import export_qonnx
    seed=20261024;random.seed(seed);np.random.seed(seed);torch.manual_seed(seed);torch.set_num_threads(4)
    bundle=Path(r'F:\CODEX\preboard_mayuan6_rc1_20261021')
    out=ROOT/'model_candidates'/'grid_offset_v3_20261024';out.mkdir(parents=True,exist_ok=True)
    rows=[json.loads(s) for s in (bundle/'dataset'/'manifest.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    cache=[]
    for row in rows:
        image=Image.open(row['image']).convert('RGB').resize((160,120),Image.Resampling.BILINEAR)
        _,expanded=rgb565_roundtrip(np.asarray(image))
        value=resize_reference(expanded).astype(np.float32).transpose(2,0,1)/255
        target=np.zeros((2,5,6,6),dtype=np.float32)
        for cid in range(2):
            boxes=[b for b in row['boxes'] if int(b['class_id'])==cid]
            if not boxes:continue
            b=max(boxes,key=lambda b:b['w']*b['h']);gx=min(5,int(b['cx']*6));gy=min(5,int(b['cy']*6))
            target[cid,:,gy,gx]=(1,b['cx']*6-gx,b['cy']*6-gy,b['w'],b['h'])
        cache.append((torch.from_numpy(value),torch.from_numpy(target),row))
    train=[v for v in cache if v[2]['split']=='training'];test=[v for v in cache if v[2]['split']=='testing']
    model=TinyQNNDetector()
    model.load_state_dict(torch.load(bundle/'model'/'tiny_qnn_detector_best.pt',map_location='cpu',weights_only=False)['model'])
    opt=torch.optim.Adam(model.parameters(),lr=0.001)
    def loss_fn(pred,t):
        pos=t[:,:,0]>0
        # Every positive cell matters; background dominates otherwise.
        raw=torch.nn.functional.binary_cross_entropy_with_logits(pred[:,:,0],t[:,:,0],reduction='none')
        pobj=(raw*pos).sum()/pos.sum().clamp_min(1)
        nobj=(raw*(~pos)).sum()/(~pos).sum().clamp_min(1)
        reg=pred[:,:,1:5].permute(0,1,3,4,2).sigmoid()
        truth=t[:,:,1:5].permute(0,1,3,4,2)
        rloss=torch.nn.functional.smooth_l1_loss(reg[pos],truth[pos]) if pos.any() else pred.sum()*0
        return pobj+2*nobj+10*rloss
    history=[]
    for epoch in range(40):
        model.train();random.shuffle(train);total=0
        for start in range(0,len(train),32):
            batch=train[start:start+32];images=torch.stack([v[0] for v in batch]);targets=torch.stack([v[1] for v in batch])
            # Horizontal flip applied consistently to image/grid/relative offset.
            if random.random()<0.5:
                images=images.flip(-1);targets=targets.flip(-1);targets[:,:,1]=1-targets[:,:,1]
            opt.zero_grad();loss=loss_fn(model(images),targets);loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),5);opt.step();total+=float(loss.detach())*len(batch)
        row={'epoch':epoch+1,'train_loss':total/len(train)};history.append(row);print(json.dumps(row),flush=True)
    model.eval();preds=[]
    with torch.no_grad():
        for start in range(0,len(test),32):preds.extend(model(torch.stack([v[0] for v in test[start:start+32]])).numpy())
    stats={n:{'ground_truth':0,'predicted':0,'matched':0,'errors':[]} for n in ['finger_tip','object']}
    empty=empty_fp=0
    for (_,_,row),pred in zip(test,preds):
        anypred=False
        for cid,name in enumerate(stats):
            s=stats[name];boxes=[b for b in row['boxes'] if int(b['class_id'])==cid]
            gt=max(boxes,key=lambda b:b['w']*b['h']) if boxes else None
            iy,ix=np.unravel_index(np.argmax(pred[cid,0]),(6,6));v=1/(1+np.exp(-np.clip(pred[cid,:,iy,ix],-80,80)))
            found=bool(v[0]>=.5);anypred|=found;s['ground_truth']+=int(gt is not None);s['predicted']+=int(found)
            if gt is not None and found:
                s['matched']+=1;s['errors'].append(float(np.hypot((ix+v[1])/6-gt['cx'],(iy+v[2])/6-gt['cy'])*96))
        if not row['boxes']:empty+=1;empty_fp+=int(anypred)
    for s in stats.values():
        errors=s.pop('errors');s['mean_center_error_pixels']=float(np.mean(errors)) if errors else None
        s['p90_center_error_pixels']=float(np.percentile(errors,90)) if errors else None
    torch.save({'model':model.state_dict(),'classes':['finger_tip','object'],'decode':'grid_relative_xy','input':'RGB565_PL_Q8_bilinear_96'},out/'candidate.pt')
    export_qonnx(model,input_t=torch.zeros(1,3,96,96),export_path=str(out/'candidate_qonnx.onnx'),dynamo=False,opset_version=13,keep_initializers_as_inputs=False)
    report={'status':'OFFLINE_CANDIDATE_ONLY_NOT_SELECTED_FOR_MOTION','decode':'cx=(grid_x+sigmoid(tx))/6; cy=(grid_y+sigmoid(ty))/6',
            'training_images':len(train),'testing_images':len(test),'epochs':40,'classes':stats,'empty_images':empty,'empty_false_positive_images':empty_fp,
            'history':history,'hardware_ip':'NOT_GENERATED','real_camera_validation':'NOT_RUN'}
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2))


if __name__=='__main__':main()
