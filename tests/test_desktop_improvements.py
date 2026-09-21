"""Kinematic convention and presentation-core invariants, independent of Qt."""

from pathlib import Path
import numpy as np
import pytest
from xml.etree.ElementTree import fromstring
from urdf2dt.app import Application
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt.kinematics import dh_fk, urdf_fk
from urdf2dt.inspection import inspect_fk
from urdf2dt.visualization.assets import geometry_node, describe, source_rgba

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "robot", ["robots/scara/scara_rrpr.urdf", "robots/ur5/ur5_serial.urdf"]
)
def test_axis_flips_preserve_physical_fk_and_repeat(robot):
    app = Application(ROOT / robot)
    original = app.run.automatic_model
    rng = np.random.default_rng(231)
    for index in range(1, len(original.rows) + 1):
        for axis in ("x", "z"):
            model = recompute_model(original, index, FrameEdit(axis_flip=axis))
            twice = recompute_model(model, index, FrameEdit(axis_flip=axis))
            combined = recompute_model(
                model, index, FrameEdit(axis_flip="z" if axis == "x" else "x")
            )
            for q in rng.uniform(-0.2, 0.2, (6, len(original.rows))):
                expected = urdf_fk(app.run.chain, q)
                for candidate in (model, twice, combined):
                    np.testing.assert_allclose(
                        dh_fk(candidate, q), expected, atol=1e-10
                    )
            assert original == app.run.automatic_model


def test_flip_session_undo_restore_and_archive(tmp_path):
    app = Application(ROOT / "robots/scara/scara_rrpr.urdf")
    for index in range(1, 5):
        app.session.unlock_next()
        app.session.propose_edit(index, FrameEdit(axis_flip="z"))
        app.session.accept()
    saved = app.session.state
    app.session.undo()
    app.session.redo()
    assert app.session.state == saved
    assert app.validate().passed
    archive = app.export(tmp_path / "session")
    assert Application.resume(archive).session.state == saved
    app.session.restore_automatic()
    assert app.session.state.working_model == app.run.automatic_model


def test_fk_physical_targets_after_flip():
    app = Application(ROOT / "robots/scara/scara_rrpr.urdf")
    model = recompute_model(app.run.automatic_model, 2, FrameEdit(axis_flip="z"))
    names = [app.run.chain.base_link] + [j.child_link for j in app.run.chain.joints]
    for target in names:
        result = inspect_fk(
            app.run.chain,
            app.run.automatic_model,
            model,
            [0.3, 0.4, 0.1, -0.2],
            names[1],
            target,
        )
        assert result["comparison"]["accepted"]["position_error_m"] < 1e-10
        assert result["comparison"]["accepted"]["orientation_error_rad"] < 1e-10


def test_visual_units_origin_and_material_are_independent(tmp_path):
    root = fromstring(
        '<robot><material name="blue"><color rgba="0 0 1 0.4"/></material></robot>'
    )
    node = fromstring(
        '<visual><geometry><mesh filename="missing.stl" scale="2 3 4"/></geometry><material name="blue"/></visual>'
    )
    effective = geometry_node(node, {"units": "mm", "xyz": "1 2 3", "rpy": "0 0 1"})
    assert effective.find("geometry/mesh").get("scale") == "0.002 0.003 0.004"
    assert node.find("geometry/mesh").get("scale") == "2 3 4"
    assert source_rgba(root, node, "visual") == (0.0, 0.0, 1.0, 0.4)
    assert describe(node, tmp_path / "robot.urdf", None, {})["status"] == "missing"
    with pytest.raises(ValueError):
        geometry_node(node, {"scale": "1 -1 1"})


def test_complete_tree_normalizes_axes_and_keeps_inactive_branch_zero():
    from urdf2dt.visualization.assets import document_poses

    root = fromstring(
        """<robot><link name="base"/><link name="a"/><link name="b"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="a"/><axis xyz="0 0 3"/></joint>
      <joint name="inactive" type="revolute"><parent link="base"/><child link="b"/><origin xyz="1 0 0"/><axis xyz="0 2 0"/></joint></robot>"""
    )
    poses = document_poses(root, {"slide": 0.25})
    np.testing.assert_allclose(np.asarray(poses["a"])[:3, 3], [0, 0, 0.25])
    np.testing.assert_allclose(np.asarray(poses["b"])[:3, 3], [1, 0, 0])
    np.testing.assert_allclose(np.asarray(poses["b"])[:3, :3], np.eye(3))


def test_asset_groups_keep_identical_meshes_with_different_materials(tmp_path):
    from urdf2dt.projects import copy_mesh

    targets = []
    for name, color in (("one", "1 0 0"), ("two", "0 0 1")):
        folder = tmp_path / name
        folder.mkdir()
        (folder / "same.obj").write_text("mtllib same.mtl\nv 0 0 0\n")
        (folder / "same.mtl").write_text("newmtl surface\nKd " + color)
        targets.append(copy_mesh(folder / "same.obj", tmp_path / "portable"))
    assert targets[0] != targets[1]
    assert (targets[0].parent / "same.mtl").read_text() != (
        targets[1].parent / "same.mtl"
    ).read_text()


def test_collada_dependencies_ignore_shader_surface_ids(tmp_path):
    from urdf2dt.projects import copy_mesh

    path = tmp_path / "material.dae"
    path.write_text(
        "<COLLADA><library_images><image><init_from>color.png</init_from></image></library_images><library_effects><effect><surface><init_from>image-id</init_from></surface></effect></library_effects></COLLADA>"
    )
    (tmp_path / "color.png").write_bytes(b"fixture")
    target = copy_mesh(path, tmp_path / "portable")
    assert (target.parent / "color.png").read_bytes() == b"fixture"


def test_named_frame_fk_analytical_offset_and_gimbal_lock():
    app = Application(ROOT / "robots/scara/scara_rrpr.urdf")
    chain = app.run.chain
    model = app.run.automatic_model
    frames = {
        "inspection_tool": {
            "link": chain.base_link,
            "xyz": [1, 2, 3],
            "rpy": [0, np.pi / 2, 0],
        }
    }
    report = inspect_fk(
        chain, model, model, [0, 0, 0, 0], chain.base_link, "inspection_tool", frames
    )
    np.testing.assert_allclose(report["position_m"], [1, 2, 3])
    assert report["orientation_warning"].startswith("Gimbal lock")
    assert report["comparison"]["accepted"]["position_error_m"] < 1e-12


def test_explicit_multiple_package_mappings(tmp_path):
    from urdf2dt.parser.robot_document import resolve_mesh

    for name in ("a", "b"):
        folder = tmp_path / name
        folder.mkdir()
        (folder / "body.stl").write_bytes(name.encode())
    mappings = {name: str(tmp_path / name) for name in ("a", "b")}
    for name in mappings:
        assert (
            resolve_mesh(
                f"package://{name}/body.stl", tmp_path / "robot.urdf", packages=mappings
            ).read_bytes()
            == name.encode()
        )


def test_single_frame_restore_after_flips_is_undoable():
    app = Application(ROOT / "robots/scara/scara_rrpr.urdf")
    for index in range(1, 5):
        app.session.unlock_next()
        app.session.propose_edit(index, FrameEdit(axis_flip="z"))
        app.session.accept()
    before = app.session.state
    app.session.restore_frame(2)
    restored = app.session.state
    for q in ([0.2, 0.3, 0.08, -0.1], [-0.3, 0.4, 0.1, 0.6]):
        np.testing.assert_allclose(
            dh_fk(restored.working_model, q), urdf_fk(app.run.chain, q), atol=1e-10
        )
    app.session.undo()
    assert app.session.state == before
    app.session.redo()
    assert app.session.state == restored
