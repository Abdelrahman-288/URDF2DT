"""Reproduce a kinematics-only world-to-tool0 UR5 fixture from the pinned source.

This is an explicit fixture preparation operation, never a validator behavior.
No origins, joint axes or limits are changed. See robots/ur5/README.md.
"""

from pathlib import Path
from hashlib import sha256
from xml.etree.ElementTree import Element, SubElement, indent, tostring

from defusedxml.ElementTree import fromstring


def main() -> None:
    root_dir = Path(__file__).resolve().parents[1]
    source = root_dir / "robots/ur5/ur5_upstream.urdf"
    data = source.read_bytes()
    if sha256(data).hexdigest() != "cede880fc2a987ef9dd23bbcda3e0ad1e7b1df8b58d638d100cfbff8ebc806cb":
        raise ValueError("Upstream fixture changed; review provenance before regenerating")
    original = fromstring(data, forbid_dtd=True, forbid_entities=True, forbid_external=True)
    by_child = {j.find("child").attrib["link"]: j for j in original.findall("joint")}
    cursor = "tool0"
    joints = []
    links = [cursor]
    while cursor != "world":
        joint = by_child[cursor]
        joints.append(joint)
        cursor = joint.find("parent").attrib["link"]
        links.append(cursor)
    robot = Element("robot", {"name": "ur5_serial"})
    for link in reversed(links):
        SubElement(robot, "link", {"name": link})
    for joint in reversed(joints):
        new_joint = SubElement(robot, "joint", dict(joint.attrib))
        for child in joint:
            if child.tag in ("parent", "child", "origin", "axis", "limit"):
                SubElement(new_joint, child.tag, dict(child.attrib))
    indent(robot, space="  ")
    header = b'<?xml version="1.0" encoding="utf-8"?>\n<!-- Derived serial fixture; see README.md and LICENSE.upstream in this directory. -->\n'
    (source.parent / "ur5_serial.urdf").write_bytes(header + tostring(robot, encoding="utf-8") + b"\n")


if __name__ == "__main__":
    main()
