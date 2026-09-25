from pathlib import Path
from xml.etree.ElementTree import fromstring
import pytest
from urdf2dt.parser.robot_document import resolve_mesh
from urdf2dt.visualization.assets import describe


def test_missing_asset_records_reference_and_actual_candidates(tmp_path):
    source = tmp_path / 'arm/urdf/robot.urdf'
    node = fromstring('<visual><geometry><mesh filename="package://arm/meshes/body.obj"/></geometry></visual>')
    record = describe(node, source, None, {})
    assert record['status'] == 'missing'
    assert record['original'] == record['requested'] == 'package://arm/meshes/body.obj'
    assert str(tmp_path / 'arm/meshes/body.obj') in record['searched']
    assert str(tmp_path / 'arm/meshes/body.stl') in record['searched']
    assert record['source_urdf'] == str(source.resolve())


def test_explicit_package_priority_and_trace_stops_at_match(tmp_path):
    chosen = tmp_path / 'chosen/meshes/body.stl'
    chosen.parent.mkdir(parents=True)
    chosen.write_text('fixture')
    searched = []
    assert resolve_mesh('package://arm/meshes/body.stl', tmp_path/'robot.urdf',
                        packages={'arm': str(tmp_path/'chosen')}, searched=searched) == chosen
    assert searched[-1] == str(chosen)
    assert len(searched) == 2


def test_rejected_uri_never_claims_searched_local_files(tmp_path):
    searched = []
    with pytest.raises(FileNotFoundError):
        resolve_mesh('package://arm/../body.stl', tmp_path/'robot.urdf', searched=searched)
    assert not searched


def test_selected_folder_can_propose_moved_absolute_file_without_changing_record(tmp_path):
    from urdf2dt.ui.mesh_help import folder_candidate
    mesh = tmp_path/'body.stl'
    mesh.write_text('fixture')
    record = {'requested': str(tmp_path/'old/body.stl'), 'source_urdf':str(tmp_path/'arm.urdf')}
    trace = []
    assert folder_candidate(record, tmp_path, trace) == mesh
    assert trace == [str(mesh)]
    assert 'old' in record['requested']
