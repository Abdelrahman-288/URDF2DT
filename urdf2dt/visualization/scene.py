"""Immutable scene geometry and an optional PyVista presentation boundary.

All coordinates are in the selected URDF base, in metres. Rendering never edits
the chain or DH model. PyVista is loaded only when drawing is requested.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from importlib import import_module
from math import dist
from pathlib import Path
from typing import Any

from urdf2dt._transforms import add, column, position, rotate, scale
from urdf2dt.dh.types import DHModel, JointType, KinematicChain, Transform, Vector
from urdf2dt.kinematics import _configuration, dh_frame_transforms, dh_fk, urdf_link_transforms


@dataclass(frozen=True, slots=True)
class SceneFrame:
    """Named base-relative rigid frame for presentation."""
    name: str
    transform: Transform


@dataclass(frozen=True, slots=True)
class SceneAxis:
    """Named joint-axis origin and unit direction in base coordinates."""
    name: str
    origin: Vector
    direction: Vector


@dataclass(frozen=True, slots=True)
class SceneBone:
    """Schematic link segment between two base-relative origins."""
    name: str
    start: Vector
    end: Vector


def _pv() -> Any:
    try:
        return import_module("pyvista")
    except ImportError as exc:
        raise RuntimeError('Static rendering requires pip install -e ".[ui]"') from exc


def draw_bone(plotter: Any, start: Vector, end: Vector, radius: float) -> Any:
    """Draw a simplified link; coincident origins need no zero-length tube."""
    if dist(start, end) <= 1e-12:
        return None
    return plotter.add_mesh(_pv().Line(start, end).tube(radius=radius), color="#627d98")


def draw_joint_axis(plotter: Any, axis: SceneAxis, length: float) -> list[Any]:
    """Gold dotted segment centred on the moving joint's axis line."""
    pv = _pv()
    actors = []
    for i in range(12):
        start = add(axis.origin, scale(axis.direction, length * (i / 12 - .5)))
        end = add(start, scale(axis.direction, length / 24))
        actors.append(plotter.add_mesh(pv.Line(start, end), color="#b77900", line_width=3))
    return actors


def draw_frame_triad(plotter: Any, transform: Transform, length: float, *,
                     urdf: bool = False, selected: bool = False) -> list[Any]:
    """RGB axes: URDF thin dashed lines; DH solid arrows and larger origins."""
    pv = _pv()
    origin = position(transform)
    actors = []
    for index, color in enumerate(("#da3849", "#16834b", "#2563db")):
        direction = column(transform, index)
        if urdf:
            for segment in range(4):
                start = add(origin, scale(direction, length * segment / 4))
                end = add(start, scale(direction, length / 7))
                actors.append(plotter.add_mesh(pv.Line(start, end), color=color, line_width=2))
        else:
            actors.append(plotter.add_mesh(pv.Arrow(start=origin, direction=direction, scale=length), color=color))
    actors.append(plotter.add_mesh(pv.Sphere(radius=length * (.065 if urdf else .10), center=origin),
                                   color="#64748b" if urdf else "#7029a8"))
    if selected:
        actors.append(plotter.add_mesh(pv.Sphere(radius=length * .23, center=origin),
                                       color="#c18a00", style="wireframe", line_width=2))
    return actors


@dataclass(frozen=True, slots=True, init=False)
class StaticScene:
    """Snapshot of URDF and DH geometry at q, suitable for headless inspection.

    selected_frame is a DH frame name (F0..Fn). Joint axes use each moving
    child origin; prismatic displacement moves this point along the same line.
    The tool frame is kept separately because Fn need not equal the URDF tip.
    """

    robot_name: str
    q: Vector
    urdf_frames: tuple[SceneFrame, ...]
    dh_frames: tuple[SceneFrame, ...]
    tool_frame: SceneFrame
    joint_axes: tuple[SceneAxis, ...]
    bones: tuple[SceneBone, ...]
    selected_frame: str | None
    frame_length: float

    def __init__(self, chain: KinematicChain, model: DHModel,
                 q: Sequence[float] | None = None, *, selected_frame: str | None = None):
        if (chain.joint_names != model.joint_names or chain.robot_name != model.robot_name
                or tuple(j.joint_type for j in chain.joints if j.joint_type != JointType.FIXED)
                != tuple(row.joint_type for row in model.rows)
                or chain.source_sha256 != model.source_sha256):
            raise ValueError("scene chain and DH model must describe the same source and ordered joints")
        values = _configuration(q if q is not None else (0.,) * len(model.rows), len(model.rows))
        poses = urdf_link_transforms(chain, values)
        dh_poses = dh_frame_transforms(model, values)
        names = (chain.base_link,) + tuple(j.child_link for j in chain.joints)
        frames = tuple(SceneFrame(f"F{i}", pose) for i, pose in enumerate(dh_poses))
        if selected_frame is not None and selected_frame not in {f.name for f in frames}:
            raise ValueError("selected_frame must name a DH frame F0..Fn")
        origins = [position(p) for p in (*poses, *dh_poses)]
        extent = dist(tuple(min(p[i] for p in origins) for i in range(3)),
                      tuple(max(p[i] for p in origins) for i in range(3)))
        data = dict(
            robot_name=chain.robot_name, q=values,
            urdf_frames=tuple(SceneFrame(name, pose) for name, pose in zip(names, poses)),
            dh_frames=frames, tool_frame=SceneFrame("DH tool", dh_fk(model, values)),
            joint_axes=tuple(SceneAxis(j.name, position(pose), rotate(pose, j.axis))
                             for j, pose in zip(chain.joints, poses[1:]) if j.joint_type != JointType.FIXED),
            bones=tuple(SceneBone(j.name, position(a), position(b))
                        for j, a, b in zip(chain.joints, poses, poses[1:])),
            selected_frame=selected_frame, frame_length=.12 * extent if extent > 1e-12 else .1,
        )
        for name, value in data.items():
            object.__setattr__(self, name, value)

    def plotter(self, *, off_screen: bool = False, urdf: bool = True, dh: bool = True,
                axes: bool = True, bones: bool = True, labels: bool = True) -> Any:
        """Build a view. Caller owns and must close the returned PyVista plotter."""
        pv = _pv()
        view = pv.Plotter(off_screen=off_screen, window_size=(1400, 1000))
        try:
            view.set_background("#f4f7fb")
            length = self.frame_length
            if bones:
                for bone in self.bones:
                    draw_bone(view, bone.start, bone.end, length * .07)
            if axes:
                for axis in self.joint_axes:
                    draw_joint_axis(view, axis, length * 2.5)
            for visible, frames, is_urdf in ((urdf, self.urdf_frames, True), (dh, self.dh_frames, False)):
                if not visible:
                    continue
                for frame in frames:
                    draw_frame_triad(view, frame.transform, length * (.7 if is_urdf else 1),
                                     urdf=is_urdf, selected=frame.name == self.selected_frame and not is_urdf)
                if labels:
                    # Presentation-only label offsets leave the actual transforms untouched.
                    points = [add(position(f.transform), (0., 0., length * (-.35 if is_urdf else .35))) for f in frames]
                    view.add_point_labels(points, [f.name for f in frames], font_size=12 if is_urdf else 16,
                                          text_color="#475569" if is_urdf else "#7029a8",
                                          show_points=False, shape=None, always_visible=True)
            view.add_text(f"URDF2DT  |  {self.robot_name}\nStatic kinematic scene", position="upper_left",
                          font_size=16, color="#172b4d")
            view.add_text("URDF: dashed RGB   |   DH: solid RGB / purple origins\n"
                          "Joint axes: dotted gold   |   Links: blue-grey\n"
                          "X red / Y green / Z blue   |   Distances in metres\n"
                          + (f"Selected: {self.selected_frame} (gold cage)" if self.selected_frame else ""),
                          position="lower_left", font_size=11, color="#334155")
            view.add_axes(viewport=(.82, .8, 1., .98))
            view.camera_position = "iso"
            view.reset_camera()
            return view
        except BaseException:
            view.close()
            raise

    def render(self, output: str | Path | None = None, **layers: bool) -> None:
        """Save an offscreen PNG, or show an interactive camera when output is None."""
        path = Path(output) if output is not None else None
        if path is not None:
            if path.suffix.lower() != ".png":
                raise ValueError("scene output must have a .png extension")
            path.parent.mkdir(parents=True, exist_ok=True)
        view = self.plotter(off_screen=path is not None, **layers)
        try:
            view.show(screenshot=str(path) if path is not None else None, auto_close=False)
        finally:
            view.close()
