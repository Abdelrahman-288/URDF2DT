# Stage 13 extension: Native robot and DH studio

The desktop application provides live robot pose sliders and constrained DH
previews in a Qt/VTK window. It shares `Application`, `EditorSession`, FK validation,
and persistence with the notebook and CLI. It does not change the stage numbering.
Stage 14 remains the research ablation stage.

## Run locally

From the repository in PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[desktop]"
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop robots/ur5/ur5_serial.urdf
```

Alternatively double-click **Launch URDF2DT.cmd**, then choose **Open URDF**.
The installed GUI entry point is `.venv\Scripts\urdf2dt-desktop.exe` on Windows.
Always use this environment: the system Python may have different dependencies.
The bundled serial fixture supports frame/kinematic inspection even when its
external meshes are unavailable; mesh notices describe missing files.

## Controls

1. Open a URDF and choose a root-to-leaf chain in the left panel. The selected
   chain alone is displayed and passed through the serial validator and DH solver.
   Changing chain starts a new editing session; save a completed session first.
2. Drag **Joint motion** sliders to move the robot immediately. Numeric fields use
   radians for revolute joints and metres for prismatic joints. Bounded joints use
   URDF limits; continuous joints have a display interval of minus to plus pi.
   Reset clamps zero to joint limits. These controls change pose, not DH geometry.
3. Orbit/zoom the 3-D view; **Fit view** resets the camera. Toggle meshes, link
   frames, Standard-DH frames, collision geometry, kinematic links, and frame names. Select a
   link to highlight mesh edges. Opacity affects visual meshes.
4. In **DH editor**, use **Unlock next**, adjust the enabled legal controls, then
   **Accept frame** or **Reject preview**. Accept can confirm an unchanged frame.
   The table and frames preview compensated DH changes while the robot retains
   its physical pose. Locked or geometrically forbidden freedoms have no controls.
5. After all frames are accepted, run **Validate FK**, then **Save session** to a
   new directory. Save independently revalidates. **Load session** restores a
   validated serial session; local mesh files must still be available separately.
6. Choose **Light** or **Dark** in the toolbar. The preference persists locally.

The DH sliders expose a practical preview interval (±0.5 m / ±pi rad); numeric
fields permit larger checked edits. These are UI ranges, not research conclusions
about the complete mathematically legal range.

**Edit URDF** opens the full loaded XML and saves a new copy before loading it.
**Mesh package directory** resolves local ROS package assets. STL and DAE mesh
geometry are supported; DAE textures and full material appearance are not preserved.
**Decompose collision (VHACD)** generates convex parts for viewing, replacing prior
generated parts on repeat. It is a synchronous batch operation; complex meshes
can take time. Generated parts are not written back into the URDF or session.

## Doctor's reference repository

The supplied [URDF-to-Digital-Twin repository](https://github.com/Ahmed-Askar-GB/URDF-to-Digital-Twin)
was inspected at commit `89314a5976b04bf3a26d4a721e22b1b6a99916d5`. It contains
`universalUR5.urdf`, `STL_Files`, and MATLAB reference scripts. Keep it as a local
ignored checkout; its source/assets are not vendored into URDF2DT. To reproduce:

```powershell
git clone https://github.com/Ahmed-Askar-GB/URDF-to-Digital-Twin.git references/doctor
git -C references/doctor checkout 89314a5976b04bf3a26d4a721e22b1b6a99916d5
.\.venv\Scripts\python.exe -m urdf2dt.ui.desktop references/doctor/universalUR5.urdf
```

The default path is `world → tool0`. The repository's URDF names DAE visual assets
that are absent from that checkout. The app uses its bundled STL files as explicit
visual substitutes and reports each substitution in the mesh-notice tooltip.
This is a visual approximation, not restoration of the missing DAE models.

The application uses **Standard DH**. The screenshot's “MDH” label is not used
because Modified DH has a different transform convention. Full branched humanoid
animation, mesh editing, and collision export are outside this extension.
MATLAB execution, report/table label reconciliation, and advisor acceptance are
still pending; inspecting the reference repository does not establish equivalence.

## Verification

Run the headless regression suite and type checker:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m mypy
.\.venv\Scripts\python.exe -m pip check
```

Run the separate native graphics check with a working desktop/display:

```powershell
.\.venv\Scripts\python.exe scripts/verify_desktop.py
```

It verifies actor reuse across 30 pose updates, actual transform changes, unchanged
DH/session state during motion, DH preview/rejection, accepted edits, sampled FK,
theme screenshots, collision decomposition/replacement, and stale-status clearing
on reload. Results and screenshots go to `outputs/tmp/desktop/`. Timings measure
Python update/render calls on this machine, not end-to-end input/display latency.
Unit tests cover branch extraction, byte-snapshot provenance, malformed trees,
entity rejection, and local mesh resolution without requiring Qt.

Verified on Windows/Python 3.12.14 with PySide6 6.11.2 and PyVista 0.49.0:
281 regression tests passed, mypy checked 33 source files, and pip reported no
broken requirements. The doctor UR5 native run reused 14 mesh actors, passed
sampled FK after accepted DH edits, and generated 56 convex collision parts.
Both theme screenshots were visually inspected. Rendering timings vary by run
and are retained in the local JSON rather than treated as a performance guarantee.

## Stage 14 handoff

Use the headless core for reproducible studies; desktop slider positions are not
experimental data. Implement and record:

- Threshold sensitivity around the default, with classification differences.
- Legal axial edit sweeps, local checks, and maximum sampled FK error per edit.
- FK sample counts of 10, 50, and 200 with recorded seeds and configurations.
- Synthetic geometry close to parallel/intersection classification boundaries.

Store input hashes, selected chain, DH convention, configurations, reference
revision, outcomes (including rejections), tables, and plots. For unbounded legal
freedoms, explicitly identify the finite tested interval; do not call a finite
sweep coverage of the entire legal domain. Preserve the provisional status of
reference labels/tolerances and separate MATLAB comparison from internal FK tests.
No Stage 14 results are claimed by this desktop extension.
