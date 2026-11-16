#!/usr/bin/env python3
"""Collect and verify a completed R2 core and all Android module providers."""
from pathlib import Path
from datetime import datetime, timezone
import fnmatch
import hashlib
import importlib.util
import json
import os
import subprocess
import sys

p = Path(os.environ['PROJECT_ROOT']).resolve()
b = Path(os.environ['BUILD_ROOT']).resolve()
root = b/'k618'
core = root/'out-gki'
device = p/'android/device/xiaomi/sheng'
out = b/'qualified'
assert not out.exists(), 'Inspect prior qualification before rerunning'
out.mkdir()
record = dict(passed=False, phase='collection', commands=[], modules_modified=False)
def sha(path):
    return hashlib.file_digest(path.open('rb'),'sha256').hexdigest()
def save():
    record['updated_utc']=datetime.now(timezone.utc).isoformat()
    (out/'verification.json').write_text(json.dumps(record,indent=2)+'\n')
def write(name,data):
    (out/name).write_text(json.dumps(data,indent=2)+'\n')
def run(args,log):
    args=list(map(str,args))
    with (out/log).open('w') as stream:
        result=subprocess.run(args,stdout=stream,stderr=subprocess.STDOUT)
    record['commands'].append(dict(argv=args,returncode=result.returncode,log=log));save()
    if result.returncode:
        print((out/log).read_text(errors='replace')[-8000:],flush=True)
    result.check_returncode()
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module
save()
try:
    release=(core/'include/config/kernel.release').read_text().strip()
    assert release=='6.18.39-android17-0-peiyuesheng-sheng-kernel',release
    collection_root=out/'modules'
    run([sys.executable,device/'tools/collect_sheng_kernel_modules.py','--source-root',root,
         '--core-build',core,'--reference-map',p/'profiles/module-sources.json',
         '--policy',device/'interfaces/kernel/6.18/module-collection.json',
         '--output',collection_root],'collection.log')
    collection=json.loads((collection_root/'collection.json').read_text())
    metadata=Path(collection['metadata_directory'])
    run(['depmod','-b',collection_root,'-F',core/'System.map','-e',release],'depmod.log')
    assert not (out/'depmod.log').read_text().strip(),'depmod diagnostics'
    elf=load('r2_elf',p/'tools/audit_kernel618_modules.py')
    modules={key:elf.ElfModule(metadata/'kernel'/row['filename'])
             for key,row in collection['modules'].items()}
    result=elf.check_candidate({m.path.name:m for m in modules.values()},
        {'source_keep_modules':['zram.ko','zsmalloc.ko']},core,
        [metadata/'kernel/zram.ko',metadata/'kernel/zsmalloc.ko'],'6.18')
    result['exact_release_for_every_module']=all(m.info['vermagic'][0].split()[0]==release for m in modules.values())
    write('abi.json',result)
    assert result['abi_passed'] and result['exact_release_for_every_module'],result['issue_counts']
    assert len(modules)==475,len(modules)
    record.update(phase='btf',module_count=len(modules),kernel_release=release);save()

    # Build the verifier from this package's own kernel libbpf sources.
    host=out/'host'
    lib=host/'libbpf'
    lib.mkdir(parents=True)
    run(['make','-C',root/'ws/common/tools/lib/bpf','OUTPUT='+str(lib)+'/',
         '-j1','BUILD_STATIC_ONLY=1','EXTRA_CFLAGS=-O2'],'libbpf.log')
    tool=host/'btf-verify'
    run(['gcc','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'ws/common/tools/lib/bpf'),
         '-I'+str(root/'ws/common/tools/include/uapi'),p/'tools/sheng-btf-distill.c',
         lib/'libbpf.a','-lelf','-lz','-o',tool],'btf-verifier.log')
    run([sys.executable,p/'tools/module-btf.py','extract','--kernel',core/'vmlinux',
         '--output',out/'verified-kernel.btf'],'kernel-btf.log')
    btf=core/'kernel-module-base.btf'
    assert btf.read_bytes()==(out/'verified-kernel.btf').read_bytes()
    btf_rows=[]
    for name,module in sorted(modules.items()):
        assert module.get('.BTF'),name
        output=subprocess.check_output([str(tool),'verify',str(btf),str(module.path)],text=True)
        assert sha(module.path)==module.digest
        btf_rows.append(dict(module=name,sha256=module.digest,types=json.loads(output)))
    write('btf.json',dict(passed=True,module_count=len(modules),kernel_btf_sha256=sha(btf),
                          tool_sha256=sha(tool),modules=btf_rows))
    record['phase']='dependencies_and_namespaces';save()
    planner=load('r2_planner',device/'tools/plan_sheng_kernel_modules.py')
    graph=planner.ModuleGraph(*[(metadata/f).read_text() for f in ('modules.dep','modules.builtin','modules.softdep')])
    excluded={'k618_load'}
    retained=set(graph.paths)-excluded
    assert all(not graph.closure([name]) & excluded for name in retained)
    order=graph.ordered(retained)
    write('dependency-graph.json',dict(passed=True,modules=len(graph.paths),retained=len(retained),
                                       excluded=sorted(excluded),order=order,unresolved_softdeps=0))
    owners={}
    for name,row in collection['modules'].items():
        directory=Path(row['path']).parent
        if directory.is_relative_to(root/'soc'):
            directory=root/'soc'
        while directory.is_relative_to(root) and not (directory/'Module.symvers').exists():
            directory=directory.parent
        assert directory.is_relative_to(root),row
        owners.setdefault(directory/'Module.symvers',set()).add(name)
    owners.setdefault(core/'Module.symvers',set()).add('vmlinux')
    records={}
    for path,names in owners.items():
        for line in path.read_text().splitlines():
            crc,symbol,provider,kind,namespace,*_=line.split('\t')
            owner=planner.module_name(provider)
            if owner not in names:
                continue
            key=(owner,symbol)
            value=(int(crc,16),namespace)
            assert key not in records or records[key]==value,key
            records[key]=value
    providers={symbol:(owner,*value) for (owner,symbol),value in records.items() if owner=='vmlinux'}
    for owner,module in modules.items():
        for symbol,crc in module.exports().items():
            expected,namespace=records[owner,symbol]
            assert crc==expected and symbol not in providers,(owner,symbol)
            providers[symbol]=(owner,crc,namespace)
    issues=[]
    imports=0
    namespaced=0
    for name,module in modules.items():
        for symbol,crc in module.versions().items():
            owner,expected,namespace=providers[symbol]
            assert crc==expected,(name,symbol)
            imports+=1
            if not namespace:
                continue
            namespaced+=1
            permitted=namespace in module.info['import_ns']
            if namespace.startswith('module:'):
                permitted |= any(fnmatch.fnmatchcase(name,v.replace('-','_')) for v in namespace[7:].split(','))
            if not permitted:
                issues.append(dict(module=name,symbol=symbol,namespace=namespace))
    write('namespaces.json',dict(passed=not issues,imports_checked=imports,namespaced_imports=namespaced,issues=issues))
    assert not issues,issues
    record.update(passed=True,phase='complete',image_sha256=sha(core/'kernel-Image'),
                  config_sha256=sha(core/'kernel.config'),modules_directory=str(metadata),
                  abi_passed=True,btf_passed=True,dependency_graph_passed=True,namespaces_passed=True,
                  signatures='Unsigned build outputs; sign when integrating into Android images.')
    save()
    print('R2 core and %d modules passed ABI, BTF, dependencies and namespaces.' % len(modules),flush=True)
except BaseException as error:
    record.update(phase='failed',error=repr(error));save()
    raise
