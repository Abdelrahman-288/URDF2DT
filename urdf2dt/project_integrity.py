"""Project-local asset closure and integrity checks, independent of machine paths."""
from hashlib import sha256
from pathlib import Path, PureWindowsPath
from urllib.parse import unquote
import json
from defusedxml.ElementTree import fromstring


def local_file(folder: Path, name: str, parent: Path | None = None) -> Path:
    """Resolve only existing project-local files; reject external/package references."""
    name = unquote(name)
    if not name or ':' in name or PureWindowsPath(name).is_absolute() or name.startswith(('\\', '/')):
        raise ValueError(f'External or invalid asset reference: {name}')
    path = ((parent or folder) / name).resolve()
    if not path.is_relative_to(folder.resolve()) or not path.is_file():
        raise ValueError(f'Missing or outside-project asset: {name}')
    return path


def file_inventory(folder: Path) -> dict[str, str]:
    return {p.relative_to(folder).as_posix(): sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob('*')) if p.is_file() and p.name != 'project.json'}


def check_integrity(folder: Path, data: dict) -> None:
    """Older projects remain readable; new manifests are checked before UI mutation."""
    inventory = data.get('file_sha256')
    if inventory is None:
        return
    if not isinstance(inventory, dict) or not {'source-original.urdf', 'robot.urdf'} <= inventory.keys():
        raise ValueError('Incomplete project integrity manifest')
    for name, digest in inventory.items():
        path = local_file(folder, name)
        if sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Project file checksum mismatch: {name}')


def check_portable(folder: Path) -> dict:
    """Check URDF/override assets and OBJ/MTL/DAE dependency closure without fallback."""
    folder = folder.resolve()
    data = json.loads((folder / 'project.json').read_text(encoding='utf-8'))
    check_integrity(folder, data)
    if not data.get('include_assets') or data.get('external_assets') or data.get('unresolved'):
        raise ValueError('Project is not self-contained; include and resolve every asset first')
    root = fromstring((folder / 'robot.urdf').read_bytes())
    pending = [local_file(folder, node.get('filename', ''))
               for node in root.findall('.//mesh') + root.findall('.//material/texture')]
    pending += [local_file(folder, v['path']) for v in data['overrides'].values() if v.get('path')]
    pending += [local_file(folder, v) for v in data.get('texture_mappings', {}).values()]
    visited: set[Path] = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        refs = []
        suffix = path.suffix.lower()
        if suffix in ('.obj', '.mtl'):
            for line in path.read_text(errors='replace').splitlines():
                parts = line.strip().split(maxsplit=1)
                if len(parts) != 2:
                    continue
                if suffix == '.obj' and parts[0] == 'mtllib':
                    refs.append(parts[1].strip())
                elif suffix == '.mtl' and (parts[0].startswith('map_') or parts[0].lower() in ('bump', 'disp', 'decal', 'refl', 'norm')):
                    ref = parts[1].strip()
                    if ref.startswith('-'):
                        raise ValueError('MTL options require explicit dependency review: ' + str(path))
                    refs.append(ref)
        elif suffix == '.dae':
            xml = fromstring(path.read_bytes())
            refs.extend(e.text.strip() for image in xml.iter() if image.tag.split('}')[-1] == 'image'
                        for e in image.iter() if e.tag.split('}')[-1] == 'init_from' and e.text
                        and not e.text.startswith('#'))
        pending.extend(local_file(folder, ref, path.parent) for ref in refs)
    inventory = data.get('file_sha256', {})
    if any(p.relative_to(folder).as_posix() not in inventory for p in visited):
        raise ValueError('Required assets are missing from the integrity manifest')
    return {'checked': True, 'asset_files': len(visited), 'file_count': len(inventory),
            'method': 'project-local URDF, override, texture and OBJ/MTL/DAE dependency closure; SHA256 integrity',
            'different_computer_tested': False}
