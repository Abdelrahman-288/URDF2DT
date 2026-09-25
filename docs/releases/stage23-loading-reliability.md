# Stage 23: robot-loading reliability

Open URDF now validates input and constructs the initial kinematic model in a
background thread while a modal progress dialog remains responsive. The current
robot is untouched during this preflight. Scene construction shows the current
geometry element and frame stage, with a Cancel button.

Cancelling preflight discards its result. Cancelling scene construction restores
the previous document, model, pose, selection, appearance, project metadata and
unsaved status using the existing replacement transaction. A failed load shows
the filename and reason, preserves the previous robot, and writes diagnostic
details to the desktop log. An invalid file can be followed by another successful
Open URDF operation.

Missing meshes remain recoverable: valid kinematics load with the available
geometry, missing-file notices and a direction to Model & Appearance → Geometry.
The mesh cache is replaced for each new robot and restored on rollback, rather
than accumulating every previously opened robot.

## Limits

- A native mesh read already in progress cannot be interrupted. Cancellation is
  checked between geometry elements and during frame construction.
- A cancelled validation thread can finish in the background; its result cannot
  install a robot. Another load is temporarily rejected until that thread exits,
  preventing unbounded accumulation of validation workers.
- This change applies to Open URDF and source-copy loading. It does not turn
  every project/session import or geometry edit into a cancellable transaction.
- Resource exhaustion or a native renderer crash cannot be recovered by Python
  exception handling. A large-robot test is evidence for that example, not an
  unlimited model-size guarantee.

## Repeatable acceptance

Run `python -m urdf2dt.ui.desktop --verify-loading NEW_FOLDER`, optionally with
`--mesh-robot robots/Atlas/urdf/atlas.urdf`. The same option is available in the
Windows executable. The check records JSON evidence and screenshots for repeated
switches, a generated tree with over 160 links, missing meshes, validation and
scene cancellation, preservation of the active robot, and recovery after an error.

The existing `--verify-switching` check additionally exercises ten Open URDF
cycles with pose/DH changes, injected scene failure and viewport gestures.

## Verified delivery — 26 September 2026

- Windows build: `d447b89`; installed in `Windows Application/URDF2DT.exe`.
- Final source regression suite: 375 passed in 192.65 seconds. Mypy passed for
  all 67 source files.
- Source ten-switch regression passed, including injected scene failure.
- Frozen loading checks passed with Atlas (66 geometry actors), generated large
  tree, missing meshes, cancellation at three stages, and error recovery.
- Frozen package checks passed for SCARA and UR5.
- Installed executable passed all ten repeated Open URDF cycles, scene-failure
  restoration and viewport gesture checks (`outputs/tmp/loading-installed-switching`).
- Local evidence: `outputs/tmp/loading-frozen-final/verification.json` and
  `outputs/tmp/loading-package-final/verification.json`.
- Build metadata correctly records a non-clean checkout because unrelated local
  settings, notebook and user assets remain present. They were not discarded or
  represented as a clean release. Separate clean-PC acceptance remains pending.

The root launcher opens the updated installation. The prior complete application
is retained at `outputs/tmp/application-before-loading-d447b89`. The illustrated
manual describes the earlier workspace build; this note is its loading addendum.
