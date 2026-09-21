"""Versioned portable kinematic projects, distinct from validated DH sessions."""

from copy import deepcopy
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
from xml.etree.ElementTree import tostring
import json
import shutil
import tempfile
import zipfile
from defusedxml.ElementTree import fromstring

from urdf2dt.app import Application
from urdf2dt.config import config_from_mapping
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.export.schemas import decode_state, decode_event
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.visualization.assets import geometry_node, add_assignments


def copy_mesh(path: Path, folder: Path) -> Path:
    """Copy one mesh/dependency group without flattening its material references."""
    digest = sha256(str(path.resolve()).encode() + path.read_bytes()).hexdigest()[:16]
    target = folder / digest / path.name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(path, target)
    dependencies = []
    if path.suffix.lower() == ".obj":
        for line in path.read_text(errors="replace").splitlines():
            if line.startswith("mtllib "):
                dependencies.append(line[7:].strip())
    if path.suffix.lower() == ".dae":
        xml = fromstring(path.read_bytes())
        dependencies.extend(
            e.text.strip()
            for image in xml.iter()
            if image.tag.split("}")[-1] == "image"
            for e in image
            if e.tag.split("}")[-1] == "init_from"
            and e.text
            and not e.text.startswith("#")
        )
    for relative in dependencies:
        dependency = (path.parent / relative).resolve()
        if (
            not dependency.is_relative_to(path.parent.resolve())
            or not dependency.is_file()
        ):
            raise ValueError(f"Unresolved/nonlocal material dependency: {relative}")
        dest = target.parent / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(dependency, dest)
        if dependency.suffix.lower() == ".mtl":
            for line in dependency.read_text(errors="replace").splitlines():
                if line.strip().startswith(("map_", "bump ")):
                    parts = line.split(maxsplit=1)
                    ref = parts[1].strip()
                    if ref.startswith("-"):
                        raise ValueError(
                            "MTL texture options need explicit dependency review before portable export"
                        )
                    texture = (dependency.parent / ref).resolve()
                    if (
                        not texture.is_relative_to(path.parent.resolve())
                        or not texture.is_file()
                    ):
                        raise ValueError(f"Missing material texture: {ref}")
                    destination = dest.parent / ref
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(texture, destination)
    return target


def save_project(editor, destination: str | Path, include_assets: bool = True):
    """Stage the complete project before publishing it to a new empty directory."""
    folder = Path(destination).resolve()
    if folder.exists() and (not folder.is_dir() or any(folder.iterdir())):
        raise ValueError("Choose a new empty project folder")
    folder.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="urdf2dt-save-", dir=folder.parent
    ) as temporary:
        staged = _write_project(editor, Path(temporary) / "project", include_assets)
        if folder.exists():
            folder.rmdir()  # Empty destination only; fails if changed concurrently.
        staged.rename(folder)
    return folder


def _write_project(editor, destination: str | Path, include_assets: bool = True):
    folder = Path(destination).resolve()
    if editor.application is None:
        raise ValueError("Load a robot first")
    if editor.application.session.pending or editor.studio.preview_before is not None:
        raise ValueError(
            "Accept or reject DH/geometry previews before saving a project"
        )
    if folder.exists() and any(folder.iterdir()):
        raise ValueError(
            "Choose a new empty project folder (existing projects are checkpointed)"
        )
    folder.mkdir(parents=True, exist_ok=True)
    app = editor.application
    source = editor.document.source if editor.document else app.run.source.source
    overrides = deepcopy(editor.studio.overrides)
    root = fromstring(source.content)
    add_assignments(root, overrides)
    unresolved = []
    texture_mappings = {}
    for link in root.findall("link"):
        for kind in ("visual", "collision"):
            for index, node in enumerate(link.findall(kind)):
                key = f"{link.get('name')}|{kind}|{index}"
                record = editor.studio.records.get(key)
                effective = geometry_node(node, overrides.get(key, {}))
                effective.attrib.pop("_mesh_units", None)
                mesh = effective.find("geometry/mesh")
                if mesh is not None:
                    if (
                        record
                        and record.get("format") == ".dae"
                        and overrides.get(key, {}).get("units", "URDF / format default")
                        != "URDF / format default"
                    ):
                        scale = [
                            float(x) / record.get("format_unit_m", 1.0)
                            for x in mesh.get("scale", "1 1 1").split()
                        ]
                        mesh.set("scale", " ".join(str(x) for x in scale))
                    if (
                        record is None
                        or not record.get("resolved")
                        or record["status"] != "loaded"
                    ):
                        unresolved.append(key)
                    elif include_assets:
                        target = copy_mesh(Path(record["resolved"]), folder / "meshes")
                        relative = target.relative_to(folder).as_posix()
                        overrides.setdefault(key, {})["path"] = relative
                        mesh.set("filename", relative)
                    else:
                        overrides.setdefault(key, {})["path"] = record["resolved"]
                        mesh.set("filename", record["resolved"])
                link.remove(node)
                link.append(effective)
    # URDF material textures are separate from embedded OBJ/DAE dependencies.
    from urdf2dt.parser.robot_document import resolve_mesh

    for texture in root.findall(".//material/texture"):
        filename = texture.get("filename", "")
        try:
            original = resolve_mesh(
                editor.texture_mappings.get(filename, filename),
                Path(app.run.chain.source_urdf or "."),
                editor.package_root,
                editor.package_mappings,
            )
            target = (
                copy_mesh(original, folder / "materials")
                if include_assets
                else original
            )
            texture.set(
                "filename",
                (
                    target.relative_to(folder).as_posix()
                    if include_assets
                    else str(target)
                ),
            )
            texture_mappings[filename] = texture.get("filename")
        except (FileNotFoundError, ValueError) as exc:
            unresolved.append(f"Material texture {filename}: {exc}")
    (folder / "source-original.urdf").write_bytes(source.content)
    (folder / "robot.urdf").write_bytes(tostring(root, encoding="utf-8"))
    for name in ("materials", "sessions", "exports"):
        (folder / name).mkdir(exist_ok=True)
    data = {
        "schema_version": "1.0",
        "kind": "urdf2dt.project",
        "source_name": source.name,
        "source_path": app.run.chain.source_urdf,
        "source_sha256": source.sha256,
        "chain_index": editor._chain_index,
        "configuration": asdict(app.run.config),
        "state": asdict(app.session.state),
        "events": [asdict(event) for event in app.session.events],
        "geometric_frames": list(app.session.geometric_frames),
        "overrides": overrides,
        "appearance": editor.studio.appearance,
        "pose": editor.q,
        "poses": editor.fk_panel.poses,
        "named_frames": editor.fk_panel.frames,
        "pose_notes": editor.fk_panel.pose_notes,
        "pose_thumbnails": editor.fk_panel.thumbnails,
        "texture_mappings": texture_mappings,
        "package_mappings": editor.package_mappings,
        "include_assets": include_assets,
        "unresolved": unresolved,
        "external_assets": not include_assets,
        "package_root": str(editor.package_root) if editor.package_root else None,
        "original_source_preserved": True,
        "scope": "Kinematic project; acceptance states are not a fresh global certificate",
    }
    (folder / "project.json").write_text(
        json.dumps(data, indent=2, allow_nan=False), encoding="utf-8"
    )
    return folder


def validate_presentation(data, app, document):
    """Reject malformed presentation input before replacing the active robot."""
    import math
    import re
    from urdf2dt.visualization.assets import numbers, UNITS
    from urdf2dt.inspection import inspect_fk

    joints = [j for j in app.run.chain.joints if j.joint_type.value != "fixed"]
    for pose in [data["pose"], *data["poses"].values()]:
        if len(pose) != len(joints) or not all(
            type(q) in (int, float) and math.isfinite(q) for q in pose
        ):
            raise ValueError(
                "Project pose must contain one finite value per movable joint"
            )
        for q, joint in zip(pose, joints):
            if joint.limit and not joint.limit.lower <= q <= joint.limit.upper:
                raise ValueError(f"Project pose exceeds limits for {joint.name}")
    for values in data["overrides"].values():
        if values.get("units", "URDF / format default") not in UNITS:
            raise ValueError("Unknown mesh units")
        for key in ("scale", "xyz", "rpy"):
            if key in values:
                numbers(values[key], 3, key == "scale")
    for values in data["appearance"].values():
        if values.get("style", "solid") not in ("solid", "wireframe", "edges"):
            raise ValueError("Unknown presentation style")
        if not 0 <= values.get("opacity", 1) <= 1:
            raise ValueError("Opacity must be between zero and one")
        if values.get("color") and not re.fullmatch(
            r"#[0-9a-fA-F]{6}", values["color"]
        ):
            raise ValueError("Invalid presentation color")
        frame = values.get("frame", {})
        for key in ("axis_size", "marker_size"):
            if not 0.005 <= frame.get(key, 0.055) <= 1:
                raise ValueError("Invalid frame/marker size")
        if (
            not 0 <= frame.get("marker_alpha", 1) <= 1
            or not 8 <= frame.get("label_size", 13) <= 40
        ):
            raise ValueError("Invalid frame display settings")
        if frame.get("marker_color") and not re.fullmatch(
            r"#[0-9a-fA-F]{6}", frame["marker_color"]
        ):
            raise ValueError("Invalid marker color")
    chain = app.run.chain
    from urdf2dt.visualization.assets import document_poses

    document_poses(document.root, {})
    for name, frame in data.get("named_frames", {}).items():
        if not name.strip():
            raise ValueError("Named frames require nonempty names")
        for key in ("xyz", "rpy"):
            if len(frame[key]) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in frame[key]
            ):
                raise ValueError("Named frame offsets require three finite numbers")
    inspect_fk(
        chain,
        app.run.automatic_model,
        app.session.state.working_model,
        data["pose"],
        chain.base_link,
        chain.base_link,
        data.get("named_frames", {}),
    )


def load_project(editor, path: str | Path):
    from urdf2dt.parser.robot_document import RobotDocument

    path = Path(path)
    path = path / "project.json" if path.is_dir() else path
    if path.stat().st_size > 50 * 1024 * 1024:
        raise ValueError("Project metadata too large")
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != "1.0" or data.get("kind") != "urdf2dt.project":
        raise ValueError("Unsupported project version")
    folder = path.parent
    raw = (folder / "source-original.urdf").read_bytes()
    if sha256(raw).hexdigest() != data["source_sha256"]:
        raise ValueError("Project source checksum mismatch")
    document = RobotDocument.load(folder / "source-original.urdf")
    selected = document.select(data["chain_index"])
    app = Application(
        URDFInput(data["source_name"], selected.content, data["source_path"]),
        config_from_mapping(data["configuration"]),
    )
    state = decode_state(data["state"])
    if state.automatic_model != app.run.automatic_model:
        raise ValueError("Project automatic baseline differs from source")
    if not validate_global_fk(
        app.run.chain, state.working_model, app.run.config
    ).passed:
        raise ValueError("Project edited FK does not match source")
    events = tuple(decode_event(event) for event in data["events"])
    if events:
        app.session = EditorSession.from_snapshot(
            state, events, tuple(data["geometric_frames"]), app.run.config.geometry
        )
    elif state != app.session.state:
        raise ValueError("Modified project state requires audit history")
    overrides = data["overrides"]
    for values in overrides.values():
        if values.get("path") and not Path(values["path"]).is_absolute():
            values["path"] = str((folder / values["path"]).resolve())
    validate_presentation(data, app, document)
    # Replace the active document only after model/schema/identity checks succeed.
    editor.document = document
    editor.application = app
    editor._chain_index = data["chain_index"]
    editor.package_mappings = data.get("package_mappings", {})
    editor.texture_mappings = {
        key: str((folder / value).resolve()) if not Path(value).is_absolute() else value
        for key, value in data.get("texture_mappings", {}).items()
    }
    editor.package_root = (
        Path(data["package_root"]) if data.get("package_root") else None
    )
    editor.studio.overrides = overrides
    editor.studio.appearance = data["appearance"]
    editor.studio.preview_before = None
    editor.studio.undo_stack = []
    editor.studio.redo_stack = []
    editor.fk_panel.frames = data.get("named_frames", {})
    editor.chain.blockSignals(True)
    editor.chain.clear()
    for p in document.paths:
        editor.chain.addItem(f"{p[0]} → {p[-1]}")
    editor.chain.setCurrentIndex(editor._chain_index)
    editor.chain.blockSignals(False)
    editor.rebuild()
    editor.set_pose(data["pose"])
    editor.fk_panel.frames = data.get("named_frames", {})
    editor.fk_panel.pose_notes = data.get("pose_notes", {})
    editor.fk_panel.thumbnails = data.get("pose_thumbnails", {})
    editor.fk_panel.rebuild()
    editor.fk_panel.poses = data["poses"]
    editor.fk_panel.presets.clear()
    editor.fk_panel.presets.addItems(data["poses"])
    editor.fk_panel.refresh_presets()
    editor.studio.dirty = False
    return folder


def portable_zip(editor, path: str | Path):
    with tempfile.TemporaryDirectory() as temp:
        folder = save_project(editor, Path(temp) / "RobotProject", True)
        data = json.loads((folder / "project.json").read_text())
        if data["unresolved"]:
            raise ValueError(
                "Resolve these dependencies before ZIP export: "
                + ", ".join(data["unresolved"])
            )
        with zipfile.ZipFile(path, "x", zipfile.ZIP_DEFLATED) as archive:
            for asset in folder.rglob("*"):
                if asset.is_file():
                    archive.write(asset, asset.relative_to(folder.parent))
