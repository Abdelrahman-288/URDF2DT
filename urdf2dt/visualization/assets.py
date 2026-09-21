"""Visual-only asset records; never modify URDF joint or DH definitions."""

from copy import deepcopy
from pathlib import Path
from xml.etree.ElementTree import SubElement
import math

from urdf2dt.parser.robot_document import resolve_mesh

SUPPORTED = {".stl", ".obj", ".ply", ".vtp", ".vtk", ".dae"}
UNITS = {"URDF / format default": 1.0, "m": 1.0, "cm": 0.01, "mm": 0.001}


def numbers(text: str, count: int, positive: bool = False) -> tuple[float, ...]:
    result = tuple(float(x) for x in text.split())
    if (
        len(result) != count
        or not all(math.isfinite(x) for x in result)
        or (positive and min(result) <= 0)
    ):
        raise ValueError(
            f"Enter {count} finite {'positive ' if positive else ''}numbers separated by spaces"
        )
    return result


def geometry_node(node, settings: dict):
    result = deepcopy(node)
    result.set("_mesh_units", settings.get("units", "URDF / format default"))
    geometry = result.find("geometry")
    if settings.get("path"):
        if geometry is None:
            geometry = SubElement(result, "geometry")
        geometry.clear()
        SubElement(geometry, "mesh", filename=settings["path"])
    mesh = result.find("geometry/mesh")
    if mesh is not None:
        scale = numbers(settings.get("scale", mesh.get("scale", "1 1 1")), 3, True)
        factor = UNITS[settings.get("units", "URDF / format default")]
        mesh.set("scale", " ".join(str(x * factor) for x in scale))
    origin = result.find("origin")
    if origin is None:
        origin = SubElement(result, "origin", xyz="0 0 0", rpy="0 0 0")
    for field in ("xyz", "rpy"):
        text = settings.get(field, origin.get(field, "0 0 0"))
        numbers(text, 3)
        origin.set(field, text)
    return result


def format_unit(path: Path) -> float:
    if path.suffix.lower() != ".dae":
        return 1.0
    from defusedxml.ElementTree import fromstring

    root = fromstring(path.read_bytes())
    for asset in root:
        if asset.tag.split("}")[-1] == "asset":
            for node in asset:
                if node.tag.split("}")[-1] == "unit":
                    value = float(node.get("meter", "1"))
                    if not math.isfinite(value) or value <= 0:
                        raise ValueError("Invalid COLLADA unit conversion")
                    return value
    return 1.0


def describe(
    node, source: Path, package: Path | None, settings: dict, packages=None
) -> dict:
    original = node.find("geometry/mesh")
    effective = geometry_node(node, settings)
    mesh = effective.find("geometry/mesh")
    origin = effective.find("origin")
    record = {
        "original": original.get("filename", "") if original is not None else "",
        "resolved": "",
        "format": "primitive",
        "status": "primitive geometry",
        "xyz": origin.get("xyz"),
        "rpy": origin.get("rpy"),
        "effective_scale": "1 1 1",
        "units": settings.get("units", "URDF / format default"),
        "message": "",
    }
    if mesh is not None:
        record["effective_scale"] = mesh.get("scale", "1 1 1")
        try:
            path = resolve_mesh(mesh.get("filename", ""), source, package, packages)
            record.update(
                resolved=str(path),
                format=path.suffix.lower(),
                status="loaded" if path.suffix.lower() in SUPPORTED else "unsupported",
            )
            record["format_unit_m"] = format_unit(path)
            factor = (
                record["format_unit_m"]
                if record["units"] == "URDF / format default"
                else 1.0
            )
            record["effective_scale"] = " ".join(
                str(x * factor) for x in numbers(record["effective_scale"], 3)
            )
        except FileNotFoundError as exc:
            record.update(status="missing", message=str(exc))
    return record


def source_rgba(root, node, kind: str):
    default = (0.62, 0.68, 0.76, 1.0) if kind == "visual" else (0.94, 0.61, 0.21, 0.3)
    material = node.find("material")
    if material is None:
        return default
    color = material.find("color")
    if color is None:
        name = material.get("name")
        for candidate in root.findall("material"):
            if candidate.get("name") == name:
                color = candidate.find("color")
                break
    return numbers(color.get("rgba"), 4) if color is not None else default


def add_assignments(root, overrides):
    for link in root.findall("link"):
        for kind in ("visual", "collision"):
            key = f"{link.get('name')}|{kind}|0"
            if not link.findall(kind) and overrides.get(key, {}).get("path"):
                node = SubElement(link, kind)
                SubElement(node, "geometry")


def document_poses(root, coordinates):
    """All rooted-tree link poses; inactive serial-path joints retain zero pose."""
    from urdf2dt._transforms import multiply, rpy_transform, axis_motion
    from urdf2dt.dh.types import IDENTITY

    joints = list(root.findall("joint"))
    children = {j.find("child").get("link") for j in joints}
    base = next(
        link.get("name")
        for link in root.findall("link")
        if link.get("name") not in children
    )
    result = {base: IDENTITY}
    while joints:
        progress = False
        for joint in list(joints):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in result:
                continue
            origin = joint.find("origin")
            axis = joint.find("axis")
            xyz = (
                numbers(origin.get("xyz", "0 0 0"), 3)
                if origin is not None
                else (0.0, 0.0, 0.0)
            )
            rpy = (
                numbers(origin.get("rpy", "0 0 0"), 3)
                if origin is not None
                else (0.0, 0.0, 0.0)
            )
            pose = multiply(result[parent], rpy_transform(xyz, rpy))
            kind = joint.get("type")
            if kind != "fixed":
                if kind not in ("revolute", "prismatic", "continuous"):
                    raise ValueError(f"Unsupported display joint: {kind}")
                direction = (
                    numbers(axis.get("xyz", "1 0 0"), 3)
                    if axis is not None
                    else (1.0, 0.0, 0.0)
                )
                length = sum(value * value for value in direction) ** 0.5
                if length < 1e-12:
                    raise ValueError("Joint axis must be nonzero")
                direction = tuple(value / length for value in direction)
                pose = multiply(
                    pose,
                    axis_motion(
                        direction,
                        coordinates.get(joint.get("name"), 0.0),
                        kind == "prismatic",
                    ),
                )
            result[child] = pose
            joints.remove(joint)
            progress = True
        if not progress:
            raise ValueError("Unresolved robot tree")
    return result
