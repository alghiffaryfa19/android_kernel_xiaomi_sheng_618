#!/usr/bin/env python3
"""Verify and copy the complete adapted source tree to a new build directory."""
import hashlib, json, os, shutil
from pathlib import Path
project = Path(os.environ['PROJECT_ROOT']).resolve()
root = Path(os.environ['BUILD_ROOT']).resolve()
manifest_path = project/'SOURCE-SHA256.json'
manifest = json.loads(manifest_path.read_text())
snapshot = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
marker = root/'.sheng618-ready.json'
if marker.exists():
    assert json.loads(marker.read_text())['snapshot_sha256'] == snapshot, 'Different source snapshot; use a new BUILD_ROOT'
    print('Prepared R2 source already present: '+str(root))
    raise SystemExit(0)
assert not root.exists() or not any(root.iterdir()), 'BUILD_ROOT must be empty'
for name, row in manifest.items():
    path = project/'sources'/name
    if 'symlink' in row:
        assert path.is_symlink() and os.readlink(path) == row['symlink'], name
    else:
        assert path.is_file() and not path.is_symlink(), name
        with path.open('rb') as stream:
            assert hashlib.file_digest(stream,'sha256').hexdigest() == row['sha256'], name
root.mkdir(parents=True, exist_ok=True)
shutil.copytree(project/'sources/k618', root/'k618', symlinks=True)
marker.write_text(json.dumps({'snapshot_sha256':snapshot, 'source_files':len(manifest)},indent=2)+'\n')
print('Prepared and verified %d R2 source entries' % len(manifest))
