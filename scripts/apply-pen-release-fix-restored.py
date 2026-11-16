#!/usr/bin/env python3
"""Apply the pen interface and release behavior patch, with pinned hashes."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


# Factory interface overlay: allow only its pinned final source, never an arbitrary edit.
def factory_interface_matches(name, got):
    from android_source_contract import android_interface_matches
    if android_interface_matches(name, got):
        return True
    path = Path(os.environ['PROJECT_ROOT']) / 'provenance/factory-interfaces.json'
    if not path.exists():
        return False
    item = json.loads(path.read_text())['files'].get(name)
    return bool(item and item['after'] == got)


project = Path(os.environ['PROJECT_ROOT']).resolve()
root = Path(os.environ['BUILD_ROOT']).resolve()
receipt = json.loads((project / 'provenance/pen-release-fix.json').read_text())
patch = project / receipt['patch']
assert hashlib.sha256(patch.read_bytes()).hexdigest() == receipt['patch_sha256']
base = json.loads((project / 'provenance/source-files.json').read_text())
for name, hashes in json.loads((project / 'provenance/charging-fix.json').read_text())['files'].items():
    base[name] = hashes['after']
repair_path = project / 'provenance/repair-r85.json'
repair = json.loads(repair_path.read_text())['files'] if repair_path.exists() else {}
def repaired(name, got):
    return name in repair and got == repair[name]['after']
diag = {'k618/ws/common/kernel/reboot.c': {'before': 'da5e1f16be3e528391692771761e98b2fad0da45e67f24e5551d60f01d0ce711', 'after': 'e44110017d8f67e8cb5ded844ec36525cf89b2c10e25a43fae4760b2d6b491f8'}, 'k618/ws/common/kernel/panic.c': {'before': '40378aae0439dbfdceae35def45d7f4075f91ba01b78359ecc1d906c7b5ad863', 'after': '4dab8155bb7b46ae8c0dc3b4d54b9c268bdb9ee4ed3f8c088f7dd3dc3ce0202f'}, 'k618/ws/common/arch/arm64/kernel/traps.c': {'before': '3e3561b2134cbbe93589f4cd7df1c7f2559ee41e25a7366644f84ae64b9a6845', 'after': '681d72ab4b92937c6126a8a58f791d003ebcb8f5f2dfce370224143b543f084d'}}
def diagnostic_matches(name, got):
    entry=diag.get(name)
    return bool(entry and got==entry['after'])
states = set()
for name, hashes in receipt['files'].items():
    assert base[name] in (hashes['before'], hashes['after']), name
    got = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if (got == hashes['after'] or factory_interface_matches(name, got) or diagnostic_matches(name, got)) or repaired(name, got):
        states.add('after')
    elif got == hashes['before']:
        states.add('before')
    else:
        raise SystemExit('Unexpected pen release source: ' + name)
assert len(states) == 1, 'Partially applied pen release patch'
if '--check' in sys.argv:
    assert states == {'after'}, 'Pen release patch has not been applied'
elif states == {'before'}:
    subprocess.run(['patch', '--batch', '--forward', '-p1', '-i', str(patch)], cwd=root, check=True)
for name, hashes in receipt['files'].items():
    got = hashlib.sha256((root / name).read_bytes()).hexdigest()
    assert (got == hashes['after'] or factory_interface_matches(name, got) or diagnostic_matches(name, got)) or repaired(name, got), name
print('Verified original pen interface with exact authorized diagnostic restart hooks')
