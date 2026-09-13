"""Adversarial XML, numeric, topology and real UR5 structural regression tests."""

from dataclasses import FrozenInstanceError
from pathlib import Path
from hashlib import sha256
import json
import subprocess
import sys

import pytest

from urdf2dt.parser.urdf_input import InputPolicy, URDFInput
from urdf2dt.parser.urdf_validator import URDFValidationError, validate_urdf

ROOT = Path(__file__).resolve().parents[1]
INVALID = ROOT / "tests/fixtures/invalid_urdf"


def robot(joint_body="", kind="continuous"):
    return (f'<robot name="test"><link name="a"/><link name="b"/>'
            f'<joint name="j" type="{kind}"><parent link="a"/><child link="b"/>'
            f'{joint_body}</joint></robot>')


def check(xml, **kwargs):
    return validate_urdf(URDFInput.from_upload("test.urdf", xml.encode()), **kwargs)


def codes(result):
    return {issue.code for issue in result.errors}


@pytest.mark.parametrize("name,expected", [
    ("malformed", "malformed_xml"), ("branching", "branching"),
    ("missing_link", "missing_link"), ("cycle", "cycle"),
    ("disconnected", "disconnected"), ("xxe", "unsafe_xml"),
    ("entity_expansion", "unsafe_xml"),
])
def test_rejected_fixtures(name, expected):
    result = validate_urdf(INVALID / f"{name}.urdf")
    assert expected in codes(result)
    assert result.document is None
    with pytest.raises(URDFValidationError) as exc:
        result.require_valid()
    assert exc.value.errors == result.errors


def test_ur5_serial_acceptance_and_full_source_rejection():
    document = validate_urdf(ROOT / "robots/ur5/ur5_serial.urdf").require_valid()
    assert (document.base_link, document.tip_link) == ("world", "tool0")
    assert len(document.links) == 9 and len(document.joints) == 8
    assert document.movable_joint_names == (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    )
    assert document.joints[1].xyz == (0, 0, 0.089159)
    with pytest.raises(FrozenInstanceError):
        document.robot_name = "changed"
    full = validate_urdf(ROOT / "robots/ur5/ur5_upstream.urdf")
    assert [e.element for e in full.errors if e.code == "branching"] == ["base_link", "wrist_3_link"]


def test_ur5_fixture_provenance_and_retained_joint_fields():
    from defusedxml.ElementTree import fromstring
    folder = ROOT / "robots/ur5"
    upstream = (folder / "ur5_upstream.urdf").read_bytes()
    derived = (folder / "ur5_serial.urdf").read_bytes()
    assert sha256(upstream).hexdigest() == "cede880fc2a987ef9dd23bbcda3e0ad1e7b1df8b58d638d100cfbff8ebc806cb"
    assert sha256(derived).hexdigest() == "2dc91dafb13f23beadb867f81880db1de4baa963973e768521ab7abbc5314882"
    source_joints = {j.attrib["name"]: j for j in fromstring(upstream, forbid_dtd=True).findall("joint")}
    for joint in fromstring(derived, forbid_dtd=True).findall("joint"):
        original = source_joints[joint.attrib["name"]]
        assert joint.attrib == original.attrib
        for child in joint:
            assert child.attrib == original.find(child.tag).attrib


def test_mixed_joint_defaults_normalization_and_fixed_retention():
    doc = validate_urdf(ROOT / "robots/examples/mixed_joints.urdf").require_valid()
    assert doc.movable_joint_names == ("hinge", "spin", "slide")
    assert doc.joints[1].axis == (0, 0, 1)
    assert doc.joints[2].axis == (1, 0, 0)
    assert doc.joints[2].xyz == (0, 0, 0) and doc.joints[2].rpy == (0, 0, 0)
    assert doc.joints[3].limit.upper == 0.5


@pytest.mark.parametrize("xml,expected", [
    ("<other/>", "invalid_robot_root"),
    ('<robot xmlns="urn:other"/>', "invalid_robot_root"),
    ('<robot name="x"/>', "no_links"),
    ('<robot name="x"><link name="a"/></robot>', "no_joints"),
    (robot().replace('name="test"', 'name=""'), "missing_robot_name"),
    (robot().replace('name="test"', 'name="test" version="1.2"'), "unsupported_version"),
    (robot().replace('name="b"', 'name="a"'), "duplicate_name"),
    (robot().replace('name="a"', 'name=" "'), "invalid_name"),
    (robot().replace('<parent link="a"/>', ''), "missing_element"),
    (robot().replace('child link="b"', 'child link=""'), "missing_reference"),
    (robot().replace('child link="b"', 'child link="a"'), "self_loop"),
    (robot(kind="floating"), "unsupported_joint_type"),
    (robot(kind="planar"), "unsupported_joint_type"),
    (robot(kind="fixed"), "no_movable_joints"),
    (robot('<mimic joint="other"/>'), "unsupported_mimic"),
    (robot('<axis xyz="0 0 0"/>'), "invalid_axis"),
    (robot('<axis/>'), "invalid_number"),
    (robot('<origin xyz="NaN 0 0"/>'), "invalid_number"),
    (robot('<origin rpy="0 inf 0"/>'), "invalid_number"),
    (robot('<origin xyz="0 0"/>'), "invalid_number"),
    (robot('<origin xyz="${height} 0 0"/>'), "invalid_number"),
    (robot('<axis xyz="1 0 0"/><axis xyz="0 1 0"/>'), "duplicate_element"),
    (robot('<origin xzy="1 0 0"/>'), "unsupported_attribute"),
    (robot('<unknown/>'), "unsupported_element"),
    (robot().replace('</robot>', '<include filename="other.urdf"/></robot>'), "unsupported_element"),
    (robot().replace('name="a"', 'name="${prefix}a"'), "unexpanded_xacro"),
])
def test_structural_failures(xml, expected):
    assert expected in codes(check(xml))


@pytest.mark.parametrize("body", [
    '', '<limit lower="0" upper="1"/>',
    '<limit lower="2" upper="1" effort="1" velocity="1"/>',
    '<limit lower="0" upper="1" effort="-1" velocity="1"/>',
    '<limit lower="0" upper="inf" effort="1" velocity="1"/>',
    '<limit lower="0" upper="1" effort="1" velocity="nan"/>',
])
@pytest.mark.parametrize("kind", ["revolute", "prismatic"])
def test_invalid_bounded_joint_limits(body, kind):
    assert not check(robot(body, kind)).valid


def test_continuous_bounds_are_not_position_limits():
    result = check(robot('<limit lower="-1" upper="1" effort="1" velocity="1"/>'))
    assert result.require_valid().joints[0].limit is None
    assert result.warnings[0].code == "ignored_position_limits"


def test_xacro_elements_and_xinclude_are_not_executed():
    for namespace in ("http://www.ros.org/wiki/xacro", "http://wiki.ros.org/xacro"):
        xml = robot().replace('<robot name="test">', f'<robot name="test" xmlns:x="{namespace}">')
        xml = xml.replace('</robot>', '<x:include filename="file:///private"/></robot>')
        assert "unexpanded_xacro" in codes(check(xml))
    xml = robot().replace('</robot>', '<include xmlns="http://www.w3.org/2001/XInclude" href="file:///private"/></robot>')
    assert "unsupported_element" in codes(check(xml))


def test_external_entities_never_read_files_or_network(tmp_path, monkeypatch):
    import builtins
    import urllib.request
    secret = tmp_path / "canary.txt"
    secret.write_text("DO_NOT_DISCLOSE", encoding="utf-8")
    payloads = [
        f'<!DOCTYPE robot [<!ENTITY secret SYSTEM "{secret.as_uri()}">]><robot name="x">&secret;</robot>',
        '<!DOCTYPE robot SYSTEM "https://example.invalid/private"><robot name="x"/>',
        '<!DOCTYPE robot [<!ENTITY a "internal">]><robot name="x"/>',
    ]
    def forbidden(*args, **kwargs):
        raise AssertionError("XML attempted external resource access")
    monkeypatch.setattr(builtins, "open", forbidden)
    monkeypatch.setattr(urllib.request, "urlopen", forbidden)
    for payload in payloads:
        result = check(payload)
        assert codes(result) == {"unsafe_xml"}
        assert "DO_NOT_DISCLOSE" not in str(result.errors)


def test_utf16_cannot_bypass_dtd_security():
    payload = '<?xml version="1.0" encoding="UTF-16"?><!DOCTYPE robot><robot name="x"/>'
    assert codes(validate_urdf(URDFInput("a.urdf", payload.encode("utf-16")))) == {"unsafe_xml"}


def test_xml_bounds():
    assert "xml_resource_limit" in codes(check(robot(), policy=InputPolicy(max_elements=3)))
    assert "xml_resource_limit" in codes(check(robot(), policy=InputPolicy(max_depth=2)))


def test_joint_order_follows_graph_not_xml_order():
    xml = ('<robot name="ordered"><link name="c"/><link name="a"/><link name="b"/>'
           '<joint name="second" type="continuous"><parent link="b"/><child link="c"/></joint>'
           '<joint name="first" type="fixed"><parent link="a"/><child link="b"/></joint></robot>')
    doc = check(xml).require_valid()
    assert doc.links == ("a", "b", "c")
    assert [j.name for j in doc.joints] == ["first", "second"]


def test_converging_edges_rejected_without_false_cycle():
    xml = ('<robot name="convergent"><link name="a"/><link name="b"/><link name="c"/>'
           '<joint name="ac" type="continuous"><parent link="a"/><child link="c"/></joint>'
           '<joint name="bc" type="fixed"><parent link="b"/><child link="c"/></joint></robot>')
    result = check(xml)
    assert "multiple_parents" in codes(result)
    assert "cycle" not in codes(result)


def test_large_serial_graph_does_not_recurse():
    count = 1100
    xml = '<robot name="long">' + ''.join(f'<link name="l{i}"/>' for i in range(count + 1))
    xml += ''.join(f'<joint name="j{i}" type="continuous"><parent link="l{i}"/><child link="l{i+1}"/></joint>' for i in range(count))
    assert len(check(xml + '</robot>').require_valid().joints) == count


@pytest.mark.parametrize("fixture,expected_exit", [("robots/ur5/ur5_serial.urdf", 0),
                                                   ("tests/fixtures/invalid_urdf/xxe.urdf", 1)])
def test_cli_json(tmp_path, fixture, expected_exit):
    result = subprocess.run([sys.executable, "-I", "-m", "urdf2dt.parser", str(ROOT / fixture), "--json"],
                            cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == expected_exit, result.stderr
    assert json.loads(result.stdout)["valid"] is (expected_exit == 0)
