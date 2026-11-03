from pathlib import Path
import datetime, hashlib, json, xml.etree.ElementTree as ET

root=Path(__file__).resolve().parents[1]
v1=root.parent/'v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((root/'reports/v1_source_baseline.json').read_text())
for rel,digest in baseline.items():
    assert sha(v1/rel)==digest, f'V1 changed: {rel}'
rtl_diff=[rel for rel,digest in baseline.items() if rel.startswith('rtl/') and sha(root/rel)!=digest]
assert rtl_diff==['rtl/vision/pick_place_target_mapper.v'],rtl_diff
for rel,digest in baseline.items():
    if rel.startswith(('config/','constraints/','model_candidates/')):
        assert sha(root/rel)==digest, f'Preserved input changed: {rel}'
tests={}
for name,marker in [('v4_radius','PASS: V4_RADIUS_30MM_36864_'),
                    ('right_to_left','PASS: COORDINATES_'),
                    ('left_to_right','PASS: COORDINATES_'),
                    ('same_bearing','PASS: COORDINATES_')]:
    log=(root/'work/coordinate_test'/f'{name}.log').read_text(errors='replace')
    lines=[line for line in log.splitlines() if line.startswith(marker)]
    assert len(lines)==1, f'Missing test pass: {name}'
    tests[name]=lines[0]
metrics=json.loads((root/'release/build_metrics.json').read_text())
assert metrics['status']=='PASS' and metrics['wns_ns']>=0 and metrics['whs_ns']>=0
for ext in ['bit','ltx','dcp']:
    assert (root/'release'/f'v4.{ext}').stat().st_size>0
project=root/'vivado/v4.xpr'
tree=ET.parse(project)
for element in tree.iter():
    path=element.get('Path','').replace('\\','/')
    assert '/vision_versions_20261029/v1/' not in path, path
manifest=[]
for folder in ['rtl','config','constraints','model_candidates','sim','scripts','tcl','release']:
    for p in sorted((root/folder).rglob('*')):
        if p.is_file() and p.name!='sha256.json':
            manifest.append({'path':p.relative_to(root).as_posix(),'sha256':sha(p)})
manifest.append({'path':'vivado/v4.xpr','sha256':sha(project)})
(root/'release/sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
report={'version':'V4','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_project':v1.as_posix(),'project':project.as_posix(),
        'bitstream':(root/'release/v4.bit').as_posix(),
        'runtime_rtl_changes':rtl_diff,'radius_mm':30,
        'v1_preserved':{'status':'PASS','checked_file_count':len(baseline)},
        'unchanged':['model','camera path','all arm control RTL and poses','coordinate and vertical ROMs','LED and button logic','second point selection'],
        'tests':{'status':'PASS','cases':tests,'evidence_scope':'synthetic model detections with actual selection logic, pose ROMs, arm FSM and byte-exact UART'},
        'build':metrics,'hardware_programmed':False,'flash_written':False,
        'physical_result':'NOT_TESTED; no hardware connected by these build/test scripts',
        'accuracy_limit':'30-mm threshold uses the existing V1 plane association and existing model finger keypoint; no new physical calibration or model accuracy claim'}
(root/'reports/delivery.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({'status':'PASS','v1_unchanged_files':len(baseline),'rtl_changed':rtl_diff,
                  'tests':list(tests),'wns_ns':metrics['wns_ns'],'whs_ns':metrics['whs_ns'],
                  'bit_sha256':sha(root/'release/v4.bit'),'hardware_programmed':False}))
