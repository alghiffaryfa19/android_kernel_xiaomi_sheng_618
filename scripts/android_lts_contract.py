# SPDX-License-Identifier: GPL-2.0-only
"""Validate the complete selected upstream overlay; reject partial application."""
import functools, hashlib, json, os
from pathlib import Path
EXPECTED_RECEIPT_SHA256 = '79221866a128924c43534e85fdd6f32f4e8c552bb45ba3d692a6fc3f287e6c00'
@functools.lru_cache(maxsize=1)
def validated_files():
    project=Path(os.environ['PROJECT_ROOT'])
    root=Path(os.environ['BUILD_ROOT'])
    path=project/'provenance/android-lts-6.18.39.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest()==EXPECTED_RECEIPT_SHA256
    receipt=json.loads(path.read_text())
    assert hashlib.sha256((project/receipt['patch']).read_bytes()).hexdigest()==receipt['patch_sha256']
    adaptation=project/'provenance/android-lts-adaptation.json'
    assert hashlib.sha256(adaptation.read_bytes()).hexdigest()=='6dd9f522677c0399b03d847ab82f61bdadc77e28169e1bda753497fa4d40193d'
    delta=json.loads(adaptation.read_text())
    assert delta['base_contract_sha256']==EXPECTED_RECEIPT_SHA256
    assert hashlib.sha256((project/delta['patch']).read_bytes()).hexdigest()==delta['patch_sha256']
    expected=dict(receipt['files'])
    for name,item in delta['files'].items():
        assert expected[name]['after']==item['before']
        expected[name]=item
    for name,item in expected.items():
        path=root/name
        got=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        assert got==item['after'], 'Incomplete or modified Android LTS source: '+name
    return expected
def android_lts_matches(name, got):
    item=validated_files().get(name)
    return item is not None and item['after']==got
if __name__=='__main__':
    print('Verified Android LTS 6.18.39: %d pinned source files' % len(validated_files()))
