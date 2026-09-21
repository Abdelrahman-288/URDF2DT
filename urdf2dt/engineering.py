"""Headless batch kinematic/asset checks; no later-stage physical models."""

from pathlib import Path
from typing import Any
from urdf2dt.app import Application
from urdf2dt.parser.robot_document import RobotDocument
from urdf2dt.dh.global_validation import validate_global_fk
from urdf2dt.visualization.assets import describe


def batch_check(paths):
    records = []
    for filename in paths:
        record: dict[str, Any] = {
            "file": str(filename),
            "valid": False,
            "paths": [],
            "assets": [],
        }
        try:
            filename = Path(filename)
            if filename.suffix.lower() == ".json":
                import json
                from hashlib import sha256
                from urdf2dt.projects import validate_presentation
                from urdf2dt.export.schemas import decode_state, decode_event
                from urdf2dt.dh.editor_session import EditorSession
                from urdf2dt.parser.urdf_input import URDFInput
                from urdf2dt.config import config_from_mapping

                data = json.loads(filename.read_text(encoding="utf-8"))
                if (
                    data.get("kind") != "urdf2dt.project"
                    or data.get("schema_version") != "1.0"
                ):
                    raise ValueError("Unsupported project schema")
                document = RobotDocument.load(filename.parent / "source-original.urdf")
                if sha256(document.source.content).hexdigest() != data["source_sha256"]:
                    raise ValueError("Source checksum mismatch")
                selected = document.select(data["chain_index"])
                app = Application(
                    URDFInput(
                        data["source_name"], selected.content, data["source_path"]
                    ),
                    config_from_mapping(data["configuration"]),
                )
                state = decode_state(data["state"])
                if state.automatic_model != app.run.automatic_model:
                    raise ValueError("Project baseline differs from source")
                events = tuple(decode_event(e) for e in data["events"])
                if events:
                    app.session = EditorSession.from_snapshot(
                        state,
                        events,
                        tuple(data["geometric_frames"]),
                        app.run.config.geometry,
                    )
                elif state != app.session.state:
                    raise ValueError("Modified state requires audit history")
                validate_presentation(data, app, document)
                record["valid"] = validate_global_fk(
                    app.run.chain, state.working_model, app.run.config
                ).passed
                for key, values in data["overrides"].items():
                    if values.get("path"):
                        path = Path(values["path"])
                        if not path.is_absolute():
                            path = filename.parent / path
                        record["assets"].append(
                            {
                                "element": key,
                                "resolved": str(path),
                                "exists": path.is_file(),
                            }
                        )
                record["scope"] = "Saved selected DH path and project asset references"
                records.append(record)
                continue
            document = RobotDocument.load(filename)
            for i, serial_path in enumerate(document.paths):
                app = Application(document.select(i))
                report = validate_global_fk(
                    app.run.chain, app.run.automatic_model, app.run.config
                )
                record["paths"].append(
                    {"links": list(serial_path), "fk_passed": report.passed}
                )
            for link in document.root.findall("link"):
                for kind in ("visual", "collision"):
                    for node in link.findall(kind):
                        status = describe(node, Path(filename), None, {})
                        record["assets"].append(
                            {"link": link.get("name"), "layer": kind, **status}
                        )
            record["valid"] = all(p["fk_passed"] for p in record["paths"])
        except Exception as exc:
            record["error"] = str(exc)
        records.append(record)
    return {
        "schema_version": "1.0",
        "scope": "Independent serial paths; asset availability is separate from kinematic validity",
        "robots": records,
    }
