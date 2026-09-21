"""Imported robot collections must resolve bodies without changing their URDFs."""

from pathlib import Path
from xml.etree.ElementTree import fromstring
import pytest
from urdf2dt.parser.robot_document import resolve_mesh
from urdf2dt.visualization.assets import source_rgba
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.parser.urdf_validator import validate_urdf


def test_drake_acceleration_preserved_without_changing_kinematic_fields():
    xml = '<robot name="arm" xmlns:drake="http://drake.mit.edu"><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="2" effort="10" velocity="3" drake:acceleration="5"/></joint></robot>'
    result = validate_urdf(URDFInput.from_upload("arm.urdf", xml.encode()))
    plain = validate_urdf(
        URDFInput.from_upload(
            "arm.urdf", xml.replace(' drake:acceleration="5"', "").encode()
        )
    )
    assert result.require_valid().joints == plain.require_valid().joints
    assert result.require_valid().source.content == xml.encode()
    assert any(w.code == "ignored_vendor_metadata" for w in result.warnings)
    typo = validate_urdf(
        URDFInput.from_upload(
            "arm.urdf", xml.replace("drake:acceleration", "velocty").encode()
        )
    )
    assert any(e.code == "unsupported_attribute" for e in typo.errors)


def asset(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"mesh fixture")
    return path.resolve()


@pytest.mark.parametrize("subdirectory", ["urdf", "robots", "model/robots"])
def test_package_sibling_meshes(tmp_path, subdirectory):
    expected = asset(tmp_path / "robots" / "robot_package" / "meshes/body.stl")
    source = tmp_path / "robots" / "robot_package" / subdirectory / "robot.urdf"
    assert resolve_mesh("package://robot_package/meshes/body.stl", source) == expected


def test_generated_file_uses_original_neighbor_package(tmp_path):
    expected = asset(tmp_path / "robots/arm/meshes/body.stl")
    source = tmp_path / "robots/xacro_generated/arm/urdf/robot.urdf"
    assert resolve_mesh("package://arm/meshes/body.stl", source) == expected


def test_renamed_owning_package_and_monorepo_prefix(tmp_path):
    expected = asset(tmp_path / "abb_irb140/meshes/irb140/visual/base.stl")
    assert (
        resolve_mesh(
            "package://abb_irb140_support/meshes/irb140/visual/base.stl",
            tmp_path / "abb_irb140/urdf/arm.urdf",
        )
        == expected
    )
    expected = asset(tmp_path / "iiwa_description/meshes/visual/link.obj")
    assert (
        resolve_mesh(
            "package://drake/manipulation/models/iiwa_description/meshes/visual/link.obj",
            tmp_path / "iiwa_description/urdf/arm.urdf",
        )
        == expected
    )


def test_shorthand_preserves_visual_collision_and_rejects_ambiguity(tmp_path):
    expected = asset(tmp_path / "arm/meshes/model_a/visual/body.stl")
    collision = asset(tmp_path / "arm/meshes/model_a/collision/body.stl")
    source = tmp_path / "arm/urdf/robot.urdf"
    assert resolve_mesh("package://visual/body.stl", source) == expected
    assert resolve_mesh("package://collision/body.stl", source) == collision
    asset(tmp_path / "arm/meshes/model_b/visual/body.stl")
    with pytest.raises(FileNotFoundError, match="Ambiguous"):
        resolve_mesh("package://visual/body.stl", source)


def test_mapping_takes_precedence_and_basename_is_not_guessed(tmp_path):
    asset(tmp_path / "robots/arm/meshes/body.stl")
    mapped = asset(tmp_path / "custom/meshes/body.stl")
    source = tmp_path / "robots/arm/urdf/robot.urdf"
    assert (
        resolve_mesh(
            "package://arm/meshes/body.stl",
            source,
            packages={"arm": str(tmp_path / "custom")},
        )
        == mapped
    )
    with pytest.raises(FileNotFoundError):
        resolve_mesh("package://arm/different/body.stl", source)


@pytest.mark.parametrize(
    "uri", ["package://arm/", "package://arm/../body.stl", "package://arm//body.stl"]
)
def test_bad_package_reference_is_an_asset_error(tmp_path, uri):
    with pytest.raises(FileNotFoundError):
        resolve_mesh(uri, tmp_path / "robot.urdf")


def test_named_inline_material_shared_by_other_links():
    root = fromstring(
        '<robot><link name="base"><visual><material name="orange"><color rgba="1 .43 0 1"/></material></visual></link><link name="arm"><visual><material name="orange"/></visual></link></robot>'
    )
    assert source_rgba(root, root.findall("link")[1].find("visual"), "visual") == (
        1.0,
        0.43,
        0.0,
        1.0,
    )
