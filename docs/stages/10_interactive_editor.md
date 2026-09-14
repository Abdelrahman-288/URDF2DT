# Stage 10: Interactive notebook editor

Launch from the repository in PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[ui]"
.\.venv\Scripts\python.exe -m jupyterlab examples/interactive_editor.ipynb
```

Run the notebook's code cell. Choose **Load path** for the prefilled UR5 fixture
or **Upload URDF** for another serial robot. Input passes through the existing
secure structural validator and automatic DH pipeline. Failed loads preserve the
current session and display an error.

## Guided workflow

1. F1 starts editable. Inspect the geometric case and available controls.
2. Enter a translation in metres or rotation in radians where allowed. Values
   are increments from the committed frame, not absolute joint coordinates.
3. **Preview** checks local geometry and displays the candidate DH table and
   scene, including adjacent compensation. It does not commit the model.
4. **Accept** commits; **Reject preview** discards. Changing an input or selected
   frame discards any previous preview, disabling Accept until another preview.
5. **Unlock next** advances in order. Revisit accepted frames using the selector.
   Changes invalidate later acceptance; the selector and table show those states.
6. **Restore frame** restores that frame; **Restore all** returns to the exact
   automatic baseline while retaining the session audit.

Frames with no continuous freedoms can still be previewed and confirmed. Locked
or out-of-order frames can be inspected but not edited. Scene and table show zero
pose; the scene offers browser camera interaction, not direct frame dragging.
State is session-owned, not inferred from widget contents. UI errors are escaped
before display, and the scene is isolated in a scripts-only sandbox iframe.

## Rendering and dependencies

The VTK scene is exported to an embedded browser viewer. Each changed preview
replaces the viewer and resets its camera. Unchanged scenes are cached. VTK text
annotations may not survive browser export; frame identity/state remains in the
selector/table. Numerical UI refresh does not imply the later global FK check.

Current PyVista uses the trame-pyvista component and VTK 9.7 requires a newer
trame-vtk. The UI extra, development requirements and Windows lock now include
compatible versions. Widget modules load only when DHEditor is constructed; core
imports remain independent of optional graphics dependencies.

## Validation and limits

Widget callback tests cover loading, all-six-frame confirmation, changed-preview
invalidation, cascades, restore, locked controls, invalid files and nonfinite
input. Tests without the optional widget dependency skip this UI test module.
The complete local environment runs it without skips. The embedded UR5 viewer
was rendered and inspected in a browser; the supplied notebook was run in JupyterLab.

Final checks: 245 tests passed, mypy clean across 26 source files, and pip check
reported no broken requirements. The browser workflow loaded UR5 and confirmed
all six frames, including a 0.06 m terminal-frame edit: the displayed d changed
from 0.0823 to 0.1423, all frames showed accepted and Unlock next was disabled.
Restore and cascade synchronization were also checked through widget callbacks.
The committed notebook has outputs cleared, so it starts a fresh session.

Reference-specific case/rule acceptance and global FK certification remain pending.
The UI says so explicitly. This stage does not add mesh loading, collision checks,
session persistence or a standalone desktop application.
