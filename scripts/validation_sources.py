"""Record simulation source and engine identity, and detect changes during a run."""
import hashlib
from pathlib import Path


def snapshot():
    paths=[]
    for root in ('src','scripts','hardware-blockers/physics'):
        paths.extend(p for p in Path(root).rglob('*') if p.is_file() and p.suffix in ('.py','.sh','.cc','.hh','.xacro','.yaml','.sdf','.stl'))
    version=Path('build/ringquad-physics/version.txt')
    if version.exists():paths.append(version)
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def drift(before,after):
    return sorted(p for p in set(before)|set(after) if before.get(p)!=after.get(p))
