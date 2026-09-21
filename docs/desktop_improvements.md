# Retained Stage 23 desktop improvement milestones

Baseline: `952d4a3` (`codex/stage23-windows-app`), inspected alongside remote
branches and worktrees. The root checkout's modified plan/notebook and untracked
robots/sessions are unrelated local work and are preserved. No Stage 24+ modules
are part of this branch. Screenshots describe desired presentation, not evidence.

## Audit

Working foundation: secure URDF tree/path selection; numerical URDF/DH FK;
constrained frame edits and global validation; validated session archives;
Qt desktop with joint sliders, primitive/STL/DAE mesh actors, basic layers,
light/dark themes and Windows packaging. Actors are reused on pose changes.

Gaps: flat structure list; no per-element asset status/replacement or units editor;
incomplete source material handling; global appearance controls only; no portable
project/geometry overrides; no dedicated FK inspector; no DH-axis flip operations;
no project recovery/checkpoints or component reports. Serial DH selection remains
explicit for branched URDFs. Dynamics and hardware work stay excluded.

## Ordered implementation and acceptance

1. **Bodies and assets.** Enumerate every visual/collision element, resolve local
   paths and explicit package roots, apply origin/scale once, preserve source
   colors/alpha and report failures beside elements. Verify real mesh-backed UR5
   and a structurally different primitive-backed SCARA; missing assets never
   become invented accurate bodies. Cache mesh data and actor identity in motion.
2. **Structure and inspector.** Searchable link/joint/geometry hierarchy, selection
   synchronization, breadcrumbs, file controls, preview/apply/cancel visual edits,
   explicit units, per-link appearance, focus/isolate and source-only inertia.
   Verify appearance does not alter accepted DH/FK; test invalid transform inputs.
3. **Frame conventions.** Proper-rotation Flip X / Flip Z with explicit companion
   axis and Standard-DH factorization checks; compensate adjacent rows/signs/tool.
   Verify repeated/combined flips and revolute/prismatic FK at multiple poses.
   Unsupported operations must explain why; preserve automatic model and history.
4. **FK calculator.** Reference/target selection, rad/deg joint input synchronized
   with sliders, homogeneous matrix, quaternion xyzw and fixed-axis XYZ RPY,
   singularity explanation, automatic/accepted comparison, copy/export and poses.
   Test analytical fixtures and distinguish one-pose evaluation from global proof.
5. **Projects.** Versioned JSON project records, original source, geometry and
   appearance, named poses/frames, edited session state and checkpoints. Save As,
   portable ZIP and asset dependency diagnostics; never overwrite original URDF.
   Verify duplicate names and relocation outside checkout/developer paths.
6. **Application quality.** Dirty-state handling, independent recovery drafts,
   presentation mode, health/explanation/measurement/component report actions,
   keyboard access, responsive progress and layout. Document measured performance.
7. **Distribution verification.** Relevant regressions/type checks plus native
   desktop feature checks; rebuild Windows EXE from recorded source revision,
   test relocated project and external assets from a different working directory.
   Keep clean-PC, genuine platform limits and unsupported material formats explicit.

These are increments within Stages 0–23, not new project stages. Completion must
be supported by test artifacts and a current executable. Industrial certification
and production readiness are not inferred from implementation or screenshots.

## Integrated desktop workflow

Open a URDF or the bundled SCARA example. The complete rooted link tree and body
geometry are displayed; select a serial path for DH editing and its live joint
controls. Other branches stay at zero pose and are not certified by that path's
global FK result. Frames and skeletons remain optional diagnostic layers.

Select a link, joint or visual/collision element in **Robot Structure**, or pick
its body in the viewport. Search and filters retain matching ancestors. Checkboxes
control visibility; **Select subtree** and Ctrl-selection support shared appearance
changes. Aliases and notes do not rename URDF identifiers. **Focus**, **Isolate**,
**Show all**, and property undo/redo are available. Undo isolation to restore the
previous visibility choices. The inspector provides separate source/joint, geometry,
appearance and frame controls. Source mass/inertia are read-only; absent inertia
is explicitly unavailable. Body and joint-marker colors are independent.

In **Geometry**, inspect the original reference, resolved filename, loading status,
source RGBA, bounding dimensions, units and effective scale. **Browse / replace /
locate mesh** opens the native chooser; selecting a link without geometry assigns
a new visual. Use Preview / Apply / Cancel for local XYZ, RPY and scale changes.
These affect geometry only. Explicit m/cm/mm units replace the format's unit
factor; COLLADA metadata is honored in format-default mode. File extensions do
not infer physical units. Package-folder selection can register multiple named
ROS packages. Missing geometry is listed rather than invented.

Mesh formats: STL, OBJ, PLY, VTK/VTP and COLLADA DAE, plus URDF box/sphere/cylinder.
Source colors and alpha are retained; OBJ/DAE materials are converted to vertex
colors by the mesh loader. Explicit URDF image textures require mesh UV coordinates.
Complex shaders, arbitrary texture graphs and general CAD formats are not supported.
Missing textures or UV coordinates produce notices and use source color. The supplied
doctor UR5 uses local STL proxies for unavailable package DAE references; those are
explicitly reported. Its local assets are used for verification, not redistributed
as newly licensed URDF2DT assets. Bundled SCARA is a labeled primitive model.

### DH flips and FK

**Flip X** postmultiplies the selected frame by `Rz(pi)`, reversing X and Y.
**Flip Z** postmultiplies it by `Rx(pi)`, reversing Y and Z. Both are proper
rotations, with determinant +1. Standard-DH factorization of adjacent transforms
is rechecked. Reversing the frame's Z changes the following movable row's joint
sign; at the terminal frame the tool transform compensates. User-facing joint
coordinates always retain the original URDF convention. The automatic model
remains immutable. Buttons are disabled when upstream acceptance or geometric
factorization disallows the operation, with a reason in the tooltip.

Preview changes the displayed DH axes/table; Accept commits it and invalidates
downstream acceptance. Reject, Undo/Redo DH, Restore frame and Restore all are
available. Bold table values differ from the automatic baseline; tooltips expose
baseline values. The automatic-frame overlay supports comparison at the same pose.
Run sampled global FK again after completing all frames. It is not a proof over
continuous configuration space. UI undo stacks last for the current open session;
saved accepted models and audit events persist, but undo stacks are not serialized.

**FK calculator** refreshes with joint motion and supports radians/degrees for
rotations and metres for translations. Select corresponding physical source/target
links or define an attached named reference/tool frame. Results include a homogeneous
matrix, position, quaternion in XYZW order, fixed-axis XYZ RPY, gimbal-lock warning,
and automatic/committed DH errors after physical-link alignment. Pending DH previews
are excluded and labeled. Copy the readable result or export complete JSON including
the rotation matrix and SI inputs. Pose presets preserve coordinates, notes and
thumbnails. Reset uses zero within legal slider limits; no unspecified home pose is
invented. Inactive-branch FK requires selecting its serial path.

### Project and interchange formats

**Save Project / Save Project As** (Ctrl+S / Ctrl+Shift+S) writes project schema 1.0:

```text
MyRobot/
  project.json           # source identity, configuration, accepted DH/audit, UI state
  source-original.urdf   # original bytes, never rewritten
  robot.urdf             # geometry-adjusted URDF, relative local asset references
  meshes/<group>/        # mesh and supported material dependency groups
  materials/<group>/     # explicit URDF texture images
  sessions/              # reserved for separately validated session archives
  exports/               # reserved for user reports/interchange
  checkpoints/           # earlier saves and optional named project snapshots
```

The asset-group hash includes source identity and mesh bytes, preventing equal
filenames or equal meshes with different materials from overwriting each other.
New projects are staged before publishing; Save creates a prior checkpoint and
replaces metadata last. OBJ MTL and simple map references, COLLADA image references,
and URDF material images are copied. Unsupported MTL options, missing files and
nonlocal dependencies block portable export with an explanation. Unchecked
**Include assets** retains external absolute paths and marks the project as external;
these drafts depend on the original files. A portable ZIP always includes assets
and refuses unresolved dependencies. Extract it before opening `project.json`.

**Export geometry URDF with assets** is separate from project saving: it exports
geometry and physical joints, not DH editor conventions or an FK certificate.
Appearance overrides, poses and frame conventions are project state. **Save session**
remains a separately validated mathematical archive. New session archives use schema
1.1 for axis-flip events; the loader accepts 1.0 and supplies its absent flip field.
Old applications are not expected to read 1.1 archives.

Unsaved changes are checked when closing or replacing the project. Geometry/DH
previews must be accepted or rejected before saving. Committed dirty drafts are
saved separately under Windows Local AppData/URDF2DT recovery every minute;
recovery never silently replaces the explicit project. Recovery drafts may retain
external assets. Named checkpoints are available from Project. F11 toggles the
presentation layout. Health checks focus asset issues; component/engineering reports
export JSON and screenshots. Batch checks accept URDFs and project records.

## Verification and limits

The numerical suite includes every-frame repeated/combined X/Z flips on UR5 and
revolute/prismatic SCARA, physical-target alignment, analytical attached-frame FK,
gimbal lock, units, duplicate asset groups, COLLADA dependencies and package mappings.
The native `--verify-studio` check exercises missing-file location, millimetre scaling,
actor reuse, unchanged DH state during appearance edits, accepted flips, project
relocation/resave, malformed-project rejection, named frames, themes and 1280x800
layout. `--verify-package` retains the previous session and collision-view checks.
Source and frozen run evidence is recorded in the distribution verification report.

Loading progress is displayed between assets. Individual native mesh decodes and
global FK checks are synchronous: very large files can temporarily block the UI.
Performance observations are development-host measurements, not guaranteed frame
rates. Native graphics tests depend on a working Windows graphics driver. Separate
clean-PC testing, broader GPU/DPI/accessibility testing, code signing, and formal
advisor acceptance are still pending. This remains a research application candidate.
