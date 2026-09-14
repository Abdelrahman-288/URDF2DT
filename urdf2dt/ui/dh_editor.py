"""Guided notebook editor. Optional widget/VTK imports occur on construction only."""

from html import escape
from importlib import import_module
from typing import Any

from urdf2dt.dh.classification import classify_dh_model, get_editable_params
from urdf2dt.dh.editor_session import EditorSession
from urdf2dt.dh.recompute import FrameEdit, recompute_model
from urdf2dt.dh.types import FrameState
from urdf2dt.dh.global_validation import validate_global_fk, GlobalFKReport
from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.pipeline import AutomaticDHResult, generate_automatic_model
from urdf2dt.visualization.scene import StaticScene
from urdf2dt.export.persistence import save_session, load_session


class DHEditor:
    """Notebook UI; EditorSession remains the sole owner of committed state.

    render_scene=False is a test hook for environments without VTK. Live browser
    scenes use an embedded VTK viewer with camera orbit/zoom and no Python server.
    """

    def __init__(self, *, render_scene: bool = True):
        w = import_module("ipywidgets")
        self._w = w
        self.render_scene = render_scene
        self.run: AutomaticDHResult | None = None
        self.session: EditorSession | None = None
        self._refreshing = False
        self._scene_key: Any = None
        self.validation_report: GlobalFKReport | None = None
        self._validated_state: Any = None
        self.controls: dict[str, Any] = {}
        self.path = w.Text(description="URDF path", placeholder="robots/ur5/ur5_serial.urdf",
                           layout=w.Layout(width="70%"))
        self.upload = w.FileUpload(accept=".urdf", multiple=False, description="Upload URDF")
        self.load_button = w.Button(description="Load path", button_style="primary")
        self.summary = w.HTML("<h2>URDF2DT · DH frame editor</h2><p>Load a serial URDF to begin.</p>")
        self.status = w.HTML()
        self.frame = w.Dropdown(description="Frame", options=[], disabled=True)
        self.case = w.HTML()
        self.parameters = w.VBox()
        self.scene = w.HTML()
        self.table = w.HTML()
        self.preview_button = w.Button(description="Preview", disabled=True)
        self.accept_button = w.Button(description="Accept", button_style="success", disabled=True)
        self.reject_button = w.Button(description="Reject preview", disabled=True)
        self.unlock_button = w.Button(description="Unlock next", disabled=True)
        self.restore_button = w.Button(description="Restore frame", disabled=True)
        self.reset_button = w.Button(description="Restore all", disabled=True)
        self.validate_button = w.Button(description="Validate FK", disabled=True)
        self.jump_button = w.Button(description="Go to flagged frame", disabled=True)
        self.validation_status = w.HTML()
        self.archive_path = w.Text(value="outputs/sessions/session", description="Session folder",
                                   style={"description_width": "initial"}, layout=w.Layout(width="70%"))
        self.save_button = w.Button(description="Save session", disabled=True)
        self.reload_button = w.Button(description="Load session")
        self.widget = w.VBox([self.summary, w.HBox([self.path, self.load_button, self.upload]),
                              self.status, self.frame, self.case, self.parameters,
                              w.HBox([self.preview_button, self.accept_button, self.reject_button]),
                              w.HBox([self.unlock_button, self.restore_button, self.reset_button]),
                              w.HBox([self.validate_button, self.jump_button]), self.validation_status,
                              w.HBox([self.archive_path, self.save_button, self.reload_button]),
                              self.scene, self.table])
        self.load_button.on_click(lambda _: self._guard(lambda: self.load(self.path.value)))
        self.upload.observe(self._uploaded, names="value")
        self.frame.observe(self._selected, names="value")
        for button, action in ((self.preview_button, self.preview), (self.accept_button, self.accept),
                               (self.reject_button, self.reject), (self.unlock_button, self.unlock),
                               (self.restore_button, self.restore), (self.reset_button, self.reset),
                               (self.validate_button, self.validate_fk), (self.jump_button, self.jump_to_failure),
                               (self.save_button, self.save), (self.reload_button, self.reload_session)):
            button.on_click(lambda _, action=action: self._guard(action))

    def _guard(self, action: Any) -> None:
        try:
            action()
        except Exception as exc:
            self.status.value = f"<p role='alert' style='color:#a32121'>{escape(str(exc))}</p>"

    def _uploaded(self, change: dict[str, Any]) -> None:
        if change["new"]:
            item = change["new"][0]
            self._guard(lambda: self.load(URDFInput.from_upload(item["name"], item["content"])))

    def load(self, source: str | URDFInput) -> None:
        run = generate_automatic_model(source)
        session = EditorSession(run.automatic_model, geometry=run.config.geometry)
        self.run, self.session = run, session
        self.summary.value = (f"<h2>{escape(run.chain.robot_name)} · DH frame editor</h2>"
                              f"<p>{len(run.chain.joints)} joints · {len(run.automatic_model.rows)} movable · "
                              f"{escape(run.chain.base_link)} → {escape(run.chain.tip_link)}</p>"
                              "<p>Zero pose · Accept all frames, then run sampled FK validation.</p>")
        self.status.value = "<p>Loaded. Preview and accept each frame in order.</p>"
        self.refresh(selected=1)

    def _selected(self, change: dict[str, Any]) -> None:
        if not self._refreshing and self.session is not None and change["new"] is not None:
            if self.session.pending is not None:
                self.session.reject("Preview discarded on frame selection.")
            self.refresh(selected=change["new"])

    def _control_changed(self, change: dict[str, Any]) -> None:
        if not self._refreshing and self.session is not None:
            if self.session.pending is not None:
                self.session.reject("Preview discarded because control values changed.")
            self.accept_button.disabled = True
            self.reject_button.disabled = True
            self.status.value = "<p>Values changed. Preview again before accepting.</p>"
            self._draw()

    def refresh(self, selected: int | None = None) -> None:
        if self.session is None:
            return
        state = self.session.state
        index = selected or self.frame.value or 1
        self._refreshing = True
        try:
            self.frame.options = [(f"F{i} · {f.value}", i) for i, f in enumerate(state.frames, 1)]
            self.frame.value = index
            self.frame.disabled = False
            assert self.run is not None
            case = classify_dh_model(state.working_model, self.run.config.geometry)[index-1]
            self.case.value = f"<b>{escape(case.case.value.title())}</b><p>{escape(case.description)}</p>"
            editable = (state.frames[index-1] in (FrameState.EDITABLE, FrameState.ACCEPTED)
                        and all(f == FrameState.ACCEPTED for f in state.frames[:index-1]))
            self.controls = {}
            for parameter in get_editable_params(case):
                control = self._w.FloatText(value=0., description=f"{parameter.name} ({parameter.unit})",
                                           style={"description_width": "initial"}, disabled=not editable)
                control.observe(self._control_changed, names="value")
                self.controls[parameter.name] = control
            self.parameters.children = tuple(self.controls.values()) or (
                self._w.HTML("<p>No continuous edit controls for this frame. Preview to confirm it.</p>"),)
            self.preview_button.disabled = not editable
            self.accept_button.disabled = self.session.pending is None
            self.reject_button.disabled = self.session.pending is None
            self.restore_button.disabled = not editable
            self.reset_button.disabled = False
            self.unlock_button.disabled = all(f == FrameState.ACCEPTED for f in state.frames)
        finally:
            self._refreshing = False
        self._draw()

    def _draw(self) -> None:
        assert self.session is not None and self.run is not None
        self.validate_button.disabled = (self.session.pending is not None or
                                        any(f != FrameState.ACCEPTED for f in self.session.state.frames))
        self.save_button.disabled = self.validate_button.disabled
        if self._validated_state is not self.session.state or self.session.pending is not None:
            self.validation_report = None
            self.validation_status.value = "<p>Current session has no sampled FK result.</p>"
            self.jump_button.disabled = True
        model = self.session.state.working_model
        pending = self.session.pending
        if pending is not None and pending.geometric_edit is not None:
            model = recompute_model(model, pending.frame_index, pending.geometric_edit, self.run.config.geometry)
        label = "Pending preview — not accepted" if pending else "Committed working model"
        rows = "".join(f"<tr><td>F{i}</td><td>{escape(r.joint_name)}</td><td>{escape(self.session.state.frames[i-1].value)}</td>"
                       f"<td>{r.a:.6g}</td><td>{r.alpha:.6g}</td><td>{r.d:.6g}</td><td>{r.theta_offset:.6g}</td></tr>"
                       for i, r in enumerate(model.rows, 1))
        self.table.value = (f"<h3>{label}</h3><table cellpadding='7'><tr><th>Frame</th><th>Joint</th><th>State</th>"
                            "<th>a (m)</th><th>alpha (rad)</th><th>d (m)</th><th>theta offset (rad)</th></tr>" + rows + "</table>")
        key = (model, self.frame.value)
        if self.render_scene and key != self._scene_key:
            view = StaticScene(self.run.chain, model, selected_frame=f"F{self.frame.value}").plotter(off_screen=True)
            try:
                html = view.trame.export_html(None)
                assert html is not None
                self.scene.value = ("<p>Drag to orbit · Scroll to zoom</p><iframe title='Robot 3-D scene' "
                                    "sandbox='allow-scripts' style='width:100%;height:520px;border:0' srcdoc=\""
                                    + escape(html.getvalue(), quote=True) + "\"></iframe>")
                self._scene_key = key
            finally:
                view.close()

    def preview(self) -> None:
        assert self.session is not None
        if self.session.pending is not None:
            self.session.reject("Replaced by a new preview.")
        self.accept_button.disabled = True
        self.reject_button.disabled = True
        try:
            self.session.propose_edit(self.frame.value, FrameEdit(**{k: v.value for k, v in self.controls.items()}))
        finally:
            self._draw()
        self.accept_button.disabled = False
        self.reject_button.disabled = False
        self.status.value = "<p>Local geometry passed. Preview includes adjacent compensation. Global validation has not run.</p>"

    def accept(self) -> None:
        assert self.session is not None
        decision = self.session.accept()
        self.status.value = f"<p>{escape(decision.reason)}</p>"
        self.refresh()

    def reject(self) -> None:
        assert self.session is not None
        self.session.reject()
        self.status.value = "<p>Preview rejected. Committed model preserved.</p>"
        self.refresh()

    def unlock(self) -> None:
        assert self.session is not None
        if self.session.pending is not None:
            self.session.reject("Preview discarded before unlocking.")
        index = self.session.unlock_next()
        self.refresh(selected=index)

    def restore(self) -> None:
        assert self.session is not None
        if self.session.pending is not None:
            self.session.reject("Preview discarded before restore.")
        self.session.restore_frame(self.frame.value)
        self.status.value = "<p>Frame restored. Downstream acceptance refreshed.</p>"
        self.refresh()

    def reset(self) -> None:
        assert self.session is not None
        self.session.restore_automatic()
        self.status.value = "<p>Exact automatic baseline restored.</p>"
        self.refresh(selected=1)

    def display(self) -> None:
        import_module("IPython.display").display(self.widget)

    def validate_fk(self) -> None:
        assert self.session is not None and self.run is not None
        if self.session.pending is not None or any(f != FrameState.ACCEPTED for f in self.session.state.frames):
            raise ValueError("Accept all frames and resolve the preview before global validation.")
        # Check automatic baseline first; a broken baseline must not certify an editor.
        baseline = validate_global_fk(self.run.chain, self.session.state.automatic_model, self.run.config)
        if not baseline.passed:
            raise ValueError("Automatic baseline failed FK validation; investigate it before the edited model.")
        report = validate_global_fk(self.run.chain, self.session.state.working_model, self.run.config)
        self.validation_report = report
        self._validated_state = self.session.state
        result = report.to_dict()
        summary = result["summary"]
        self.validation_status.value = (f"<h3>Sampled FK: {'PASS' if report.passed else 'FAIL'}</h3>"
            f"<p>{len(result['samples'])} samples; max position {summary['max_position_error_m']:.3g} m; "
            f"max orientation {summary['max_orientation_error_rad']:.3g} rad.</p>"
            f"<p>{escape(result['diagnostic']['message'])}</p><p>Provisional tolerances; sampled evidence, not a continuous-space proof.</p>")
        self.jump_button.disabled = result["diagnostic"]["first_frame"] is None

    def jump_to_failure(self) -> None:
        if self.validation_report is None:
            raise ValueError("No current FK report.")
        frame = self.validation_report.to_dict()["diagnostic"]["first_frame"]
        if frame is not None:
            self.frame.value = frame

    def save(self) -> None:
        assert self.run is not None and self.session is not None
        path = save_session(self.archive_path.value, self.run, self.session)
        self.status.value = f"<p>Validated session saved: {escape(str(path))}</p>"

    def reload_session(self) -> None:
        from pathlib import Path
        path = Path(self.archive_path.value)
        loaded = load_session(path / "session.json" if path.is_dir() else path)
        self.run, self.session = loaded.run, loaded.session
        self._validated_state = self.session.state
        self.validation_report = loaded.report
        self.summary.value = f"<h2>{escape(self.run.chain.robot_name)} · Reloaded DH session</h2>"
        self.status.value = "<p>Session reloaded; sampled FK validation reproduced.</p>"
        self.validation_status.value = "<p>Reloaded session: sampled FK PASS under saved tolerances.</p>"
        self.refresh(selected=1)
