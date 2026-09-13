"""Scene coordinates are testable without the optional graphics stack."""

from dataclasses import FrozenInstanceError, replace
from pathlib import Path

import pytest

from urdf2dt._transforms import position
from urdf2dt.pipeline import generate_automatic_model
from urdf2dt.visualization.scene import StaticScene, draw_bone

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def run():
    return generate_automatic_model(ROOT / "robots/ur5/ur5_serial.urdf")


def test_zero_pose_origins_and_layers(run):
    scene = StaticScene(run.chain, run.automatic_model, selected_frame="F2")
    origins = {f.name: position(f.transform) for f in scene.urdf_frames}
    assert origins["shoulder_link"] == pytest.approx((0, 0, .089159), abs=1e-10)
    assert origins["upper_arm_link"] == pytest.approx((0, .13585, .089159), abs=1e-10)
    assert origins["tool0"] == pytest.approx((.81725, .19145, -.005491), abs=1e-10)
    assert len(scene.urdf_frames) == 9
    assert len(scene.bones) == 8
    assert len(scene.joint_axes) == 6
    assert len(scene.dh_frames) == 7
    assert position(scene.tool_frame.transform) == pytest.approx(origins["tool0"], abs=1e-10)
    assert scene.joint_axes[0].direction == (0, 0, 1)
    assert scene.joint_axes[1].direction == (0, 1, 0)


def test_nonzero_pose_and_snapshot(run):
    q = [1.5707963267948966, 0, 0, 0, 0, 0]
    scene = StaticScene(run.chain, run.automatic_model, q)
    q[0] = 0
    origins = {f.name: position(f.transform) for f in scene.urdf_frames}
    assert origins["upper_arm_link"] == pytest.approx((-.13585, 0, .089159), abs=1e-10)
    assert scene.joint_axes[1].direction == pytest.approx((-1, 0, 0), abs=1e-10)
    assert scene.q[0] != 0
    with pytest.raises(FrozenInstanceError):
        scene.selected_frame = "F3"


def test_tool_alignment_is_not_last_dh_frame(run):
    tool = ((1., 0., 0., .1), (0., 1., 0., .2), (0., 0., 1., .3), (0., 0., 0., 1.))
    model = replace(run.automatic_model, tool_transform=tool)
    scene = StaticScene(run.chain, model)
    assert scene.tool_frame.transform != scene.dh_frames[-1].transform


@pytest.mark.parametrize("q", [(0,), (float("nan"),) * 6])
def test_invalid_coordinates(run, q):
    with pytest.raises(ValueError):
        StaticScene(run.chain, run.automatic_model, q)


def test_reject_wrong_model_and_selection(run):
    with pytest.raises(ValueError, match="same source"):
        StaticScene(run.chain, replace(run.automatic_model, source_sha256="a" * 64))
    with pytest.raises(ValueError, match="selected_frame"):
        StaticScene(run.chain, run.automatic_model, selected_frame="F99")


def test_degenerate_bone_does_not_load_renderer():
    assert draw_bone(None, (0., 0., 0.), (0., 0., 0.), .01) is None


def test_wrong_output_extension(run, tmp_path):
    with pytest.raises(ValueError, match=".png"):
        StaticScene(run.chain, run.automatic_model).render(tmp_path / "bad.jpg")


def test_prismatic_axis_at_displaced_child():
    run = generate_automatic_model(ROOT / "robots/examples/mixed_joints.urdf")
    scene = StaticScene(run.chain, run.automatic_model, (0, 0, .3))
    axis = scene.joint_axes[-1]
    assert axis.name == "slide"
    assert axis.origin == pytest.approx((0, .3, .1))
    assert axis.direction == pytest.approx((0, 1, 0))
    assert scene.bones[-1].end == axis.origin
