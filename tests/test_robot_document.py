"""Desktop tree selection must preserve a valid, independently owned serial input."""

from pathlib import Path
from xml.etree.ElementTree import fromstring

import pytest
from defusedxml.common import DefusedXmlException

from urdf2dt.app import Application
from urdf2dt.ui.robot_document import RobotDocument, resolve_mesh


def write_robot(tmp_path, body):
    path = tmp_path / "tree.urdf"
    path.write_text(f'<robot name="tree">{body}</robot>', encoding="utf-8")
    return path


def test_selected_branch_preserves_visuals_and_source_snapshot(tmp_path):
    path = write_robot(
        tmp_path,
        """
      <material name="blue"><color rgba="0 0 1 1"/></material>
      <link name="base"/>
      <link name="tool0"><visual><geometry><box size="1 2 3"/></geometry></visual></link>
      <link name="other"/>
      <joint name="drive" type="continuous"><parent link="base"/><child link="tool0"/><axis xyz="0 0 1"/></joint>
      <joint name="aux" type="fixed"><parent link="base"/><child link="other"/></joint>
    """,
    )
    document = RobotDocument.load(path)
    assert document.paths[0] == ("base", "tool0")
    original = document.source.content
    selected = document.select(0)
    root = fromstring(selected.content)
    assert [link.get("name") for link in root.findall("link")] == ["base", "tool0"]
    assert root.find("link/visual/geometry/box") is not None
    assert root.find("material") is not None
    assert selected.source_path == str(path.resolve())
    assert selected.sha256 != document.source.sha256
    assert Application(selected).run.chain.joint_names == ("drive",)
    path.write_text("not XML", encoding="utf-8")
    assert document.select(0) == selected
    assert document.source.content == original
    assert len(document.root.findall("joint")) == 2


@pytest.mark.parametrize(
    "body",
    [
        '<link name="a"/><link name="a"/>',
        '<link name="a"/><link name="b"/>',
        '<link name="a"/><joint name="x"><parent link="a"/><child link="missing"/></joint>',
        '<link name="a"/><joint name="x"><parent link="a"/><child link="a"/></joint>',
        '<link name="a"/><link name="b"/><joint name="x"><parent link="a"/><child link="b"/></joint><joint name="y"><parent link="a"/><child link="b"/></joint>',
        '<link name="a"/><link name="b"/><link name="c"/><joint name="x"><parent link="b"/><child link="c"/></joint><joint name="y"><parent link="c"/><child link="b"/></joint>',
    ],
)
def test_invalid_tree_rejected(tmp_path, body):
    with pytest.raises(ValueError):
        RobotDocument.load(write_robot(tmp_path, body))


def test_entities_are_rejected(tmp_path):
    path = tmp_path / "entity.urdf"
    path.write_text(
        '<!DOCTYPE robot [<!ENTITY x SYSTEM "file:///secret">]><robot name="x"><link name="&x;"/></robot>'
    )
    with pytest.raises(DefusedXmlException):
        RobotDocument.load(path)


def test_mesh_resolution_and_explicit_stl_proxy(tmp_path):
    source = tmp_path / "robot.urdf"
    mesh = tmp_path / "link.stl"
    mesh.touch()
    assert resolve_mesh("link.stl", source) == mesh.resolve()
    package = tmp_path / "description"
    (package / "meshes").mkdir(parents=True)
    asset = package / "meshes/link.stl"
    asset.touch()
    uri = "package://description/meshes/link.stl"
    assert resolve_mesh(uri, source, tmp_path) == asset.resolve()
    assert resolve_mesh(uri, source, package) == asset.resolve()
    fallback = tmp_path / "STL_Files"
    fallback.mkdir()
    proxy = fallback / "wrist1.stl"
    proxy.touch()
    assert (
        resolve_mesh("package://description/visual/wrist_1.dae", source, fallback)
        == proxy.resolve()
    )
    with pytest.raises(FileNotFoundError):
        resolve_mesh("https://example.com/link.stl", source)
    with pytest.raises(FileNotFoundError):
        resolve_mesh("package://description/visual/wrist_1.dae", source)
