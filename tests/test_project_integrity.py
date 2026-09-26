import json
from pathlib import Path
import pytest
from urdf2dt.project_integrity import check_integrity, check_portable, file_inventory


def project(folder):
    (folder / 'source-original.urdf').write_text('<robot name="test"/>')
    (folder / 'robot.urdf').write_text('<robot name="test"><link name="base"><visual><geometry><mesh filename="body.obj"/></geometry></visual></link></robot>')
    (folder / 'body.obj').write_text('mtllib body.mtl\nv 0 0 0\n')
    (folder / 'body.mtl').write_text('newmtl body\nmap_Kd color.png\n')
    (folder / 'color.png').write_bytes(b'texture fixture')
    data = {'include_assets':True, 'external_assets':False, 'unresolved':[], 'overrides':{},
            'file_sha256':file_inventory(folder)}
    (folder / 'project.json').write_text(json.dumps(data))
    return data


def test_asset_closure_and_relocation(tmp_path):
    source = tmp_path / 'original'; source.mkdir()
    project(source)
    moved = source.rename(tmp_path / 'relocated')
    assert check_portable(moved)['asset_files'] == 3
    assert not check_portable(moved)['different_computer_tested']


@pytest.mark.parametrize('name', ['color.png', 'body.obj', 'robot.urdf'])
def test_missing_and_corrupt_files(tmp_path, name):
    data = project(tmp_path)
    (tmp_path/name).write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='checksum'):check_integrity(tmp_path,data)
    (tmp_path/name).unlink()
    with pytest.raises(ValueError, match='Missing'):check_portable(tmp_path)


@pytest.mark.parametrize('reference', ['../outside.png', 'C:/assets/color.png', 'package://assets/color.png'])
def test_external_texture_rejected_even_with_new_manifest(tmp_path, reference):
    data = project(tmp_path)
    (tmp_path / 'body.mtl').write_text('map_Kd '+reference)
    data['file_sha256'] = file_inventory(tmp_path)
    (tmp_path/'project.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):check_portable(tmp_path)


def test_external_project_not_portable(tmp_path):
    data=project(tmp_path); data['external_assets']=True
    (tmp_path/'project.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='not self-contained'):check_portable(tmp_path)


def test_legacy_manifest_optional(tmp_path):
    check_integrity(tmp_path, {})
