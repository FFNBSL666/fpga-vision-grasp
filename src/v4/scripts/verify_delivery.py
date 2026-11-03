from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[1]
for e in json.loads((ROOT/'release/sha256.json').read_text(encoding='utf-8')):
    assert hashlib.sha256((ROOT/e['path']).read_bytes()).hexdigest()==e['sha256'],e['path']
status=json.loads((ROOT/'reports/delivery.json').read_text(encoding='utf-8'))
assert status['build']['status']=='PASS'
assert status['tests']['status']=='PASS'
print('PASS: candidate bitstream, probes, RTL and ROM hashes match the tested release')
