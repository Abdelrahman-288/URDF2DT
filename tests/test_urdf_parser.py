"""Parser coordinate conventions, joint retention, and immutable source snapshot."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from urdf2dt.config import EditorConfig, GeometryConfig
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import validate_urdf
from urdf2dt.dh.types import JointType

ROOT = Path(__file__).resolve().parents[1]


def test_parser_keeps_chain_order_fixed_joints_limits_and_digest():
    doc = validate_urdf(ROOT / "robots/examples/mixed_joints.urdf").require_valid()
    chain = SerialURDFParser().parse(doc, EditorConfig())
    assert (chain.base_link, chain.tip_link) == ("world", "tool")
    assert chain.joint_names == ("hinge", "spin", "slide")
    assert chain.joints[0].joint_type == JointType.FIXED
    assert chain.joints[1].limit.lower == -1.5
    assert chain.joints[2].axis == (1, 0, 0)
    assert chain.source_sha256 == doc.source.sha256
    assert chain.source_urdf == doc.source.source_path


def test_noncommuting_rpy_uses_urdf_convention_and_not_threshold_snapping():
    xml = ('<robot name="rotation"><link name="base"/><link name="tip"/>'
           '<joint name="j" type="continuous"><parent link="base"/><child link="tip"/>'
           '<origin xyz="1 2 3" rpy="0.3 -0.5 0.8"/><axis xyz="1 2 3"/></joint></robot>')
    doc = validate_urdf(URDFInput.from_upload("test.urdf", xml.encode())).require_valid()
    config = EditorConfig()
    chain = SerialURDFParser().parse(doc, config)
    matrix = np.array(chain.joints[0].origin)
    np.testing.assert_allclose(matrix[:3, :3], Rotation.from_euler("xyz", [.3, -.5, .8]).as_matrix(), atol=1e-15)
    np.testing.assert_allclose(matrix[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(chain.joints[0].axis, np.array([1, 2, 3]) / np.sqrt(14))
    assert SerialURDFParser().parse(doc, replace(config, geometry=GeometryConfig(parallel_threshold=.1))) == chain


def test_parser_uses_validated_snapshot_after_file_changes(tmp_path):
    path = tmp_path / "test.urdf"
    path.write_bytes((ROOT / "robots/examples/mixed_joints.urdf").read_bytes())
    doc = validate_urdf(path).require_valid()
    path.write_bytes(b"malformed replacement")
    chain = SerialURDFParser().parse(doc, EditorConfig())
    assert chain.source_sha256 == doc.source.sha256
    assert chain.joint_names == ("hinge", "spin", "slide")
    with pytest.raises(TypeError):
        SerialURDFParser().parse(path, EditorConfig())
