#!/usr/bin/env python3
"""Apply only the two reviewed pen driver files on a verified r87 base."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

project = Path(os.environ['PROJECT_ROOT']).resolve()
root = Path(os.environ['BUILD_ROOT']).resolve()
receipt = json.loads((project / 'provenance/pen-stability.json').read_text())
patch = project / receipt['patch']
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
diag = {'k618/ws/common/kernel/reboot.c': {'before': 'da5e1f16be3e528391692771761e98b2fad0da45e67f24e5551d60f01d0ce711', 'after': 'e44110017d8f67e8cb5ded844ec36525cf89b2c10e25a43fae4760b2d6b491f8'}, 'k618/ws/common/kernel/panic.c': {'before': '40378aae0439dbfdceae35def45d7f4075f91ba01b78359ecc1d906c7b5ad863', 'after': '4dab8155bb7b46ae8c0dc3b4d54b9c268bdb9ee4ed3f8c088f7dd3dc3ce0202f'}, 'k618/ws/common/arch/arm64/kernel/traps.c': {'before': '3e3561b2134cbbe93589f4cd7df1c7f2559ee41e25a7366644f84ae64b9a6845', 'after': '681d72ab4b92937c6126a8a58f791d003ebcb8f5f2dfce370224143b543f084d'}}

assert sha(patch) == receipt['patch_sha256'], 'Pen patch hash mismatch'
for name, expected in receipt['unchanged_files'].items():
    if name in diag:
        assert expected == diag[name]['before'], name
        expected = diag[name]['after']
    assert sha(root / name) == expected, 'Protected r87 source differs: ' + name
states = set()
for name, entry in receipt['files'].items():
    path = root / name
    assert path.resolve().is_relative_to(root), path
    got = sha(path)
    if got == entry['after']:
        states.add('after')
    elif got == entry['before']:
        states.add('before')
    else:
        raise RuntimeError('Unexpected pen driver source: ' + name)
assert len(states) == 1, 'Partial pen stability source patch'
if '--check' in sys.argv:
    assert states == {'after'}, 'Pen patch has not been applied'
elif states == {'before'}:
    subprocess.run(['patch', '--batch', '--forward', '-p1', '-i', str(patch)], cwd=root, check=True)
for name, entry in receipt['files'].items():
    assert sha(root / name) == entry['after'], name
for name, expected in receipt['unchanged_files'].items():
    if name in diag:
        assert expected == diag[name]['before'], name
        expected = diag[name]['after']
    assert sha(root / name) == expected, 'Protected r87 source changed: ' + name
print('Verified pen stability patch; exact opt-in diagnostic hooks verified; pen and filter sources unchanged', flush=True)
