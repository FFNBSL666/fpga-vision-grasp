"""Run official FINN v0.10.1 QONNX conversion offline, record real blockers."""
from pathlib import Path
import json
import os
import shutil
import sys
import traceback

ROOT=Path(__file__).resolve().parents[1]
FINN=ROOT/'work'/'finn_source'/'finn-0.10.1'
sys.path[:0]=[str(ROOT/'work'/'finn_onnx_compat'),
              str(ROOT/'work'/'finn_source'/'qonnx-fd61cfeebbdaba351abf7e9d54cd785d7776fa4f'/'src'),str(FINN/'src')]


def main():
    import numpy as np
    from qonnx.core.modelwrapper import ModelWrapper
    from qonnx.core.onnx_exec import execute_onnx
    from qonnx.transformation.infer_shapes import InferShapes
    from qonnx.transformation.general import GiveUniqueNodeNames
    from qonnx.util.cleanup import cleanup_model
    from finn.transformation.qonnx.convert_qonnx_to_finn import ConvertQONNXtoFINN
    pairs=[('rc1',Path(r'F:\CODEX\preboard_mayuan6_rc1_20261021\model\tiny_qnn_detector_qonnx.onnx')),
           ('grid_offset_v3',ROOT/'model_candidates'/'grid_offset_v3_20261024'/'candidate_qonnx.onnx')]
    report={'compiler_source':'Xilinx/finn v0.10.1','conversion':[],
            'environment':{'docker':shutil.which('docker'),'wsl_linux_installed':False,'vivado_windows':str(Path(r'F:\Vivado\2018.3\bin\vivado.bat')),
                           'vitis_hls_linux':None},'stitched_ip':'NOT_GENERATED','full_model_bitstream':'NOT_GENERATED'}
    for name,path in pairs:
        row={'candidate':name,'source':str(path)}
        try:
            before=cleanup_model(ModelWrapper(str(path))).transform(GiveUniqueNodeNames()).transform(InferShapes())
            after=before.transform(ConvertQONNXtoFINN()).transform(InferShapes())
            dest=ROOT/'model_candidates'/name;dest.mkdir(parents=True,exist_ok=True)
            after.save(str(dest/'finn_converted.onnx'))
            ops={}
            for node in after.graph.node:ops[node.op_type]=ops.get(node.op_type,0)+1
            row.update(status='PASS',converted=str(dest/'finn_converted.onnx'),ops=ops)
            rng=np.random.default_rng(20261024);inp=rng.random((1,3,96,96),dtype=np.float32)
            a=next(iter(execute_onnx(before,{before.graph.input[0].name:inp}).values()))
            b=next(iter(execute_onnx(after,{after.graph.input[0].name:inp}).values()))
            error=float(np.max(np.abs(a-b)))
            row['conversion_test_max_abs_error']=error;row['conversion_test']='PASS' if error<1e-4 else 'FAIL'
        except Exception as exc:
            row.update(status='BLOCKED',error=str(exc),trace=traceback.format_exc())
        report['conversion'].append(row)
        print(json.dumps(row,indent=2),flush=True)
    dest=ROOT/'reports'/'finn_conversion_20261024.json';dest.write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__':main()
