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
