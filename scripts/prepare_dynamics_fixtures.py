"""Print reproducible dynamics fixtures derived from pinned local sources."""

from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
from xml.etree.ElementTree import fromstring, SubElement, indent, tostring


def fixtures() -> dict[str, str]:
    root = Path(__file__).resolve().parents[1]
    upstream = (root / "robots/ur5/ur5_upstream.urdf").read_bytes()
    if sha256(upstream).hexdigest() != "cede880fc2a987ef9dd23bbcda3e0ad1e7b1df8b58d638d100cfbff8ebc806cb":
        raise ValueError("Pinned UR5 input changed")
    inertias = {link.attrib["name"]: link.find("inertial") for link in fromstring(upstream).findall("link")}
    ur5 = fromstring((root / "robots/ur5/ur5_serial.urdf").read_bytes())
    ur5.set("name", "ur5_dynamics")
    for link in ur5.findall("link"):
        supplied = inertias.get(link.attrib["name"])
        if supplied is not None:
            link.append(deepcopy(supplied))
    scara = fromstring((root / "robots/scara/scara_rrpr.urdf").read_bytes())
    scara.set("name", "scara_dynamics_nominal")
    parameters = {"base": (5., "0 0 -0.1"), "upper_arm": (2., "0.175 0 0"),
                  "forearm": (1.5, "0.125 0 0.035"), "carriage": (.8, "0 0 0.08"),
                  "wrist": (.3, "0 0 -0.06"), "tool": (.2, "0.015 0 -0.02")}
    for link in scara.findall("link"):
        if link.attrib["name"] not in parameters:
            continue
        mass, xyz = parameters[link.attrib["name"]]
        inertial = SubElement(link, "inertial")
        SubElement(inertial, "mass", value=str(mass))
        SubElement(inertial, "origin", xyz=xyz, rpy="0.2 -0.1 0.3")
        SubElement(inertial, "inertia", ixx="0.01", iyy="0.012", izz="0.008",
                   ixy="0.001", ixz="-0.0005", iyz="0.0004")
    result = {}
    for name, robot in (("ur5_dynamics.urdf", ur5), ("scara_dynamics.urdf", scara)):
        indent(robot, space="  ")
        result[name] = '<?xml version="1.0"?>\n' + tostring(robot, encoding="unicode") + "\n"
    return result


if __name__ == "__main__":
    print(json.dumps(fixtures()))
