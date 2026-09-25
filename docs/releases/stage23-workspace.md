# Stage 23 desktop workspace delivery — 2026-09-25

Source: `da8b32bab03cd6c1cd8b49acd8c9dfaa49b642b0` (clean build checkout).
Scope: retained Stages 0–23. No removed dynamics/control/hardware stages restored.

## Delivered behavior

- Maximized normal startup, with F11 presentation mode retained.
- Body clicks select the physical link, expand/scroll/highlight the component tree, highlight a matching DH row, and show a moving golden world-aligned bounding box. Background click clears selection; camera drag does not.
- Hierarchy on the left; Model & Appearance inspector and tool selector on the right; resizable/collapsible DH, FK and validation results below the viewport.
- Joint type, limits, numeric input, slider and individual reset. Joint/DH/FK angle and length display choices preserve SI model values; table/joint precision and layout preferences persist separately.
- Shared URDF/automatic DH/committed DH/preview model selection for FK and geometric Jacobian analysis. Missing previews clear their results.
- Geometric J, Jv/Jw grids with explicit joint columns, joint-rate inputs, relative twist, normalized rank/SVD/condition, full-precision SI JSON, and expanded analysis dialog.
- Camera presets, fit robot/selection, optional ground grid, active-chain annotations, isolation that restores prior visibility, theme/layer menus, Save As asset option, and stale-validation indication.

## Evidence

- 375 pytest tests passed, including 11 new finite-difference/invalid-scale checks for relative Jacobians on UR5 and SCARA, moving references, attached frames, signed DH and prismatic joints.
- Mypy passed for 65 source files.
- Native workspace tests: maximization, moving selection box, synchronized tree, isolation restoration, 20 display-unit toggles preserving q/model, unmodified spin/form precision, rate input without pose mutation, model synchronization and missing-preview clearing, selection preserving pending DH preview, grid visibility.
- Native expanded dialog and Save As asset-option checks passed; screenshots reviewed.
- Native Qt scale-factor exercises at 1.0, 1.25 and 1.5; window sizes requested at 1366×768 and 1920×1080, both themes. These are logical Qt window sizes, not three independently verified physical Windows monitor configurations.
- Packaged checks: recorded below after final execution. These run from Downloads outside the repository with Python/Git removed from PATH.

## Limits

The broader UI plan remains an incremental acceptance record; see `../UI_Improvement_Plan.md`. Automatic frame-axis sizing, frozen DH identifier columns, per-joint configurable step values, and persistence of tool/collapsed-panel choice remain refinements. Existing symbolic displays and source-geometry fields retain explicit formats rather than adopting every global display preference. The Jacobian is geometric, not an RPY-rate Jacobian. Rank diagnostics use the documented characteristic length and relative threshold; they are not proof of global controllability or FK correctness.

Atlas was tested with local user-supplied assets, including explicitly identified STL substitutes for missing OBJ files. These assets are not redistributed in the application ZIP. Whole-body rendering does not imply whole-tree DH editing.

A separate clean Windows PC, physical multi-monitor/DPI accessibility checks, and industrial certification have not been completed. The code was committed locally; this delivery does not claim a GitHub push or release publication.

## Final packaged delivery

All seven additional frozen checks passed: workspace, presentation with Atlas, repeated switching, studio/persistence, numbers, symbolic FK, and symbolic Jacobian. The base packaged launch/validation/session checks also passed.

Executable: `C:/Users/abdel/OneDrive/Desktop/URDF2DT/Windows Application/URDF2DT.exe`.

Share the entire ZIP, not the executable alone: `Windows Application/URDF2DT-v1.0-Windows.zip` (208,948,308 bytes). SHA-256: `6a337ee305852acaff3b7c27dbf4614adf68aa44a7d40d45963e5ed8f1fd7eb4`. The relocated bundle verified 1617 manifest files. Reports and selected screenshots are beside the installed app.

Independent extracted delivery and all GUI reports: `C:/Users/abdel/Downloads/URDF2DT-Workspace-da8b32b`.

Original pre-workspace app backup: `outputs/tmp/windows-before-workspace-6f73e2f`. Intermediate candidate backup: `outputs/tmp/windows-before-workspace-da8b32b`.
