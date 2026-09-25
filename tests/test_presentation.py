from pathlib import Path
import pytest
from urdf2dt.parser.robot_document import resolve_mesh
from urdf2dt.ui.matrix_view import compact_expression
from urdf2dt.symbolic import symbolic_jacobian
from urdf2dt.dh.types import DHModel, DHRow


def test_obj_stl_exact_location_and_original_priority(tmp_path):
    folder = tmp_path/'Atlas'/'urdf'
    folder.mkdir(parents=True)
    meshes = folder/'meshes'
    meshes.mkdir()
    stl = meshes/'body.stl'
    stl.touch()
    uri = 'package://Atlas/urdf/meshes/body.obj'
    source = folder/'atlas.urdf'
    assert resolve_mesh(uri,source) == stl.resolve()
    obj = meshes/'body.obj'
    obj.touch()
    assert resolve_mesh(uri,source) == obj.resolve()
    assert resolve_mesh('meshes/body.obj',source) == obj.resolve()
    with pytest.raises(FileNotFoundError): resolve_mesh('package://Atlas/other/body.obj',source)
    with pytest.raises(FileNotFoundError): resolve_mesh('package://Atlas/urdf/meshes/missing.obj',source)


def test_readable_preview_retains_symbols_and_tiny_terms():
    assert compact_expression('pi/2 + 0.250000000000001*q1 + 6.123233995736765e-17*t100') == 'pi/2 + 0.250*q1 + 6.123e-17*t100'
    model = DHModel('planar',(DHRow(.123456789,0,0,0,'joint'),),'test')
    result = symbolic_jacobian(model)
    original = result.documents()
    readable = result.documents(include_layout=True)
    assert readable[:2] == original
    assert len(readable[2]['matrix']) == 6
    assert readable[2]['joint_key'] == ['q1: joint [rad]']
    assert '.123456789' in original[0]
