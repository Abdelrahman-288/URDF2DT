# URDF2DT Desktop UI Improvement Plan

Created: 2026-09-25  
Status: Core workspace increment implemented; native verification completed; packaged verification recorded in the release record. Remaining acceptance items are listed below.

## Purpose and scope

Make the desktop application easier to understand, inspect, and present by improving layout, numerical readability, robot navigation, and feedback. The reference is the supplied screenshot showing an Atlas robot, a crowded structure/property panel, joint controls, and a DH table.

These are UI improvement milestones within the retained Stage 0–23 project scope. The UI stage identifiers below do not replace the master plan's stage numbering or restore removed development stages. Professional quality must be supported by verification; this plan does not claim industrial certification.

Before implementation, locate the latest retained implementation branch/worktree and audit the actual application. Screenshots are design references, not proof of implemented capabilities. Preserve unrelated local work.

## Design principles

- Give each panel one primary responsibility.
- Make the robot and essential numerical results the visual priorities.
- Separate display preferences from physical model data and accepted DH state.
- Keep units, precision, selection, and status language consistent.
- Distinguish the complete robot from the serial chain being edited.
- Expose advanced details on demand rather than displaying every control at once.
- Preserve the existing mathematical core, immutable automatic baseline, validation gates, and session compatibility.

## Stage overview

| Stage | Deliverable | Depends on | Status |
|---|---|---|---|
| UI-1 | Clear workspace layout | Implementation audit | Planned |
| UI-2 | Shared engineering-number presentation | UI-1 layout decisions | Planned |
| UI-3 | Readable DH results table | UI-1, UI-2 | Planned |
| UI-4 | Compact, precise joint controls | UI-1, UI-2 | Planned |
| UI-5 | Clear Robot Structure and active-chain navigation | UI-1 | Planned |
| UI-6 | Improved viewport and camera controls | UI-1, UI-5 | Planned |
| UI-7 | Consistent menus, themes, accessibility, and feedback | UI-1–UI-6 | Planned |
| UI-8 | Integrated verification and Windows delivery | UI-1–UI-7 | Planned |

Prioritize layout, numerical formatting, and the DH table before further visual polish.

## Feature workspaces

The application should provide dedicated workspaces instead of showing every control at once:

| Workspace | Primary purpose | Main result |
|---|---|---|
| Model & Appearance | Inspect links, meshes, materials, and visibility | Selected robot geometry and asset status |
| Joint Motion | Move the active chain and save poses | Current joint configuration and pose |
| DH Editor | Edit legal frame freedoms and conventions | Previewed or accepted DH model |
| Forward Kinematics | Calculate transforms for a selected configuration | Pose transform and comparison |
| Jacobian | Analyze local motion, sensitivity, and singularities | Jacobian matrix and diagnostics |
| Validation | Run and inspect global checks | Versioned validation report |

Switching workspaces must preserve the loaded robot, current pose, selection, accepted editor state, and pending-preview rules. Each workspace may rearrange panels, but it must use the same authoritative model and unit/precision settings.

## UI-1 — Workspace layout and panel responsibilities

### Tasks

- Reserve the left panel for robot hierarchy, search, filters, and active-chain selection.
- Give the center viewport the largest practical area.
- Move selection properties to the right-side inspector, alongside consistently navigated tools for Joint Motion, DH Editor, and Forward Kinematics where available.
- Use one tool-navigation pattern; avoid redundant selectors controlling the same mode.
- Place DH Table, FK Results, and Validation in a resizable, collapsible bottom results area.
- Make panel splitters easy to discover and provide Reset Layout.
- Avoid nesting a scrolling property form inside an already scrolling left panel.
- Preserve panel sizes and selected tool as user preferences without changing model state.
- Keep selection inspection from silently discarding a pending DH preview.

### Target organization

```text
File   Edit   View   Tools   Help
[Open Project / URDF] [Save] [Undo] [Redo]       Robot: Atlas

Structure             3D Robot View                  Active tool
Search and filters                                  Joint Motion
Robot hierarchy                                     DH Editor
Active-chain context                                Forward Kinematics
                                                    Selection Inspector

                      View controls
                      Collapsible results:
                      DH Table | FK Results | Validation

Ready                  Geometry: 50/50 loaded        Active chain: 4 joints
```

Do not introduce decorative buttons for unavailable functionality. Preserve working existing entry points and represent incomplete features explicitly.

### Acceptance criteria

- The tree, active tool, viewport, and results can be resized without overlap or clipped essential controls.
- Collapsing the results area meaningfully enlarges the viewport.
- Properties no longer crowd the structure tree.
- Tool switching and layout changes preserve the robot pose, accepted edits, and pending-edit semantics.

## UI-2 — Engineering numbers, units, and precision

### Tasks

- Implement shared number formatting for joint inputs, DH values, transforms, and validation results.
- Use right-aligned numerical columns and tabular digits.
- Provide visible angle-unit selection (degrees/radians) and length-unit selection (millimetres/metres).
- Use consistent precision within a quantity/column and allow configurable display precision.
- Show units in a dedicated field suffix or column header, not ambiguous mixed-unit labels alone.
- Use scientific notation for small nonzero validation residuals; never round them into an apparent exact pass or zero.
- Normalize rounded negative zero in ordinary displayed model values.
- Offer full stored precision through tooltips and copy actions. Distinguish copying displayed values from copying full-precision values.
- Optionally show recognized angular forms such as pi/2 as secondary annotations using a documented comparison tolerance. Do not alter values to match symbolic forms.
- Keep canonical internal units and schema-defined export precision unchanged. Label any display-unit table export with its units and convention.
- Switching units must not commit a rounded replacement value or mutate the model.

### Presentation examples

| Quantity | Example presentation | Rule |
|---|---|---|
| Joint angle | 0.1000 rad or 5.73 degrees | Explicit selectable units |
| DH twist | 1.5708 rad; optional pi/2 annotation | Consistent angle precision |
| Link distance | 162.000 mm or 0.162000 m | Explicit length units |
| Validation residual | 2.4 × 10^-8 rad | Preserve visibility of small errors |
| Joint-coordinate sign | +1 or -1 | Explain meaning in tooltip |

### Acceptance criteria

- Unit switching changes presentation only; FK, DH values, and stored configuration remain numerically unchanged.
- Repeated unit changes and focus changes do not accumulate rounding drift.
- Small nonzero residuals, large values, negative values, and near-zero values remain readable.
- Copy/export behavior makes units and precision unambiguous.

## UI-3 — DH table readability

### Tasks

- Increase row height and improve header contrast.
- Freeze identifying columns where practical so frame/joint names remain visible during horizontal scrolling.
- Right-align numerical values and apply UI-2 formatting consistently.
- Highlight the selected frame and distinguish values changed from the automatic baseline without relying only on color.
- Use compact state labels such as Locked, Editing, and Accepted, mapped to actual session states.
- Present joint-coordinate signs as +1/-1 in an optional advanced column.
- Add descriptive parameter and convention tooltips.
- Provide Copy Table and Export actions with explicit units and precision.
- Preserve the distinction between accepted data and preview data.
- Keep results expandable for detailed review and collapsible for geometry inspection.

### Acceptance criteria

- Users can identify the selected frame, its state, and changed parameters at a glance.
- Headers, signs, units, and numbers remain readable at supported display scales.
- Table values stay synchronized with preview, accept, reject, restore, and pose-independent DH state.
- Table exports do not masquerade as validated session exports.

## UI-4 — Compact and precise joint controls

### Tasks

- Use compact rows containing joint identity, type, slider, numerical input, unit, and reset action.
- Show original URDF identifiers alongside or behind friendly display names.
- Display bounded minimum/maximum values and provide an adjustable input step.
- Synchronize sliders and numerical fields without feedback loops or accumulated rounding.
- Give clear feedback at limits; preserve existing validated input handling.
- Label continuous-joint slider intervals as UI ranges rather than physical limits.
- Distinguish Reset to Zero, where valid, from a named Home pose. Clearly explain any limit-clamped reset behavior.
- Label controls with their active-chain context and movable-joint count.

### Example row

```text
J1  Back rotation                          Revolute
[-90 degrees] -----------o----------- [+90 degrees]
Value: [5.73] degrees    Step: [0.1]    [Reset]
```

### Acceptance criteria

- Rotary and prismatic controls display appropriate units and limits.
- Slider and numerical entry produce the same physical pose.
- Invalid input leaves the previous valid pose intact and explains the problem.
- Manipulating joint controls does not change accepted DH geometry.

## UI-5 — Robot Structure and active-chain clarity

### Tasks

- Use consistent link/joint/geometry icons rather than generic folder styling.
- Reduce repeated status text such as Link, revolute, and loaded; retain details in the inspector and tooltips.
- Show prominent badges primarily for warnings or actionable states, with text explanations.
- Keep visual/collision children collapsed by default and retain search/filter behavior.
- Add per-item visibility controls and contextual Focus, Isolate, Hide, and Inspect Geometry actions.
- Restore previous visibility when leaving isolation mode.
- Synchronize selection between tree, viewport, and applicable DH-table entries.
- Clearly label the complete robot and the selected base-to-tip DH chain separately.
- Highlight active-chain members and show the movable-joint count near the joint controls and table.
- Selecting an off-chain component may inspect its geometry; it must not silently change the edited chain.
- Retain explicit session handling when changing the active chain.

### Acceptance criteria

- On a branched robot such as Atlas, users can explain why a whole body may be visible while only a selected chain appears in the DH table.
- Off-chain links are not presented as having DH rows in the current chain.
- Tree and viewport selection agree; visibility/isolation operations preserve the kinematic model.
- Missing geometry can be located from a visible warning without searching every property panel.

## UI-5A — Forward Kinematics workspace

### Tasks

- Add a dedicated Forward Kinematics workspace reachable from the main tool navigation.
- Let the user choose the source URDF model, automatic DH model, accepted edited DH model, or a clearly labeled preview model.
- Select the target link/frame and reference frame from searchable lists. Show the active chain and joint ordering beside the inputs.
- Provide joint inputs with explicit units, limits, zero/home presets, and synchronization with Joint Motion.
- Display the homogeneous 4×4 transform, position, rotation matrix, quaternion with component order, and roll/pitch/yaw convention.
- Show the transform path and distinguish physical link poses from DH display-frame poses.
- Compare URDF FK with automatic and accepted edited-DH FK at the same physical target, showing position and orientation differences separately.
- Provide Copy Results and Export Results actions containing source identity, model identity, frames, joint values, units, and precision.
- Mark preview calculations as provisional and invalidate stale comparisons after relevant edits.

### Acceptance criteria

- The selected target and reference frames are unambiguous.
- Results update correctly after joint changes and agree with the existing kinematics implementation and independent fixtures.
- A legal DH frame convention change does not change the physical pose reported for the same target.
- Degree/metre display choices do not change canonical internal calculations.
- FK calculation at one pose remains distinct from sampled global validation.

## UI-5B — Jacobian workspace

### Purpose

The Jacobian workspace describes how joint-rate changes produce instantaneous end-effector linear and angular velocity. It is a kinematic analysis feature; it does not add dynamics or control functionality.

### Tasks

- Add a dedicated Jacobian workspace with the same model, target-frame, reference-frame, and joint-order selectors used by Forward Kinematics.
- Let users choose geometric or analytical Jacobian representation where both are supported, and document the convention.
- Display the 6×n Jacobian with labeled rows (`vx`, `vy`, `vz`, `wx`, `wy`, `wz`) and labeled joint columns.
- Provide separate linear and angular blocks, with units of metres per radian or metres per metre for linear velocity and radians per radian or radians per metre for angular velocity as applicable.
- Show the current joint configuration and allow direct joint-rate inputs to calculate end-effector twist.
- Display the resulting linear velocity, angular velocity, and the relationship `twist = J(q) · q̇`.
- Provide singularity diagnostics: rank, singular values, condition number where meaningful, and a clearly labeled warning threshold.
- Show a manipulability measure only when its convention and units are documented.
- Support Copy Matrix and Export Analysis with source/model identity, frame names, joint values, joint rates, units, convention, and numerical precision.
- Highlight selected rows/columns and explain joint contributions without changing robot state.
- Keep Jacobian calculations synchronized with the FK workspace and Joint Motion pose, while ensuring rate inputs do not move the robot unless the user explicitly applies a pose update.

### Acceptance criteria

- Matrix dimensions and joint-column ordering match the selected active chain.
- Revolute and prismatic columns use the correct units and geometric interpretation.
- Jacobian values agree with independent analytical or finite-difference checks within documented tolerances.
- A frame flip or DH edit updates the displayed convention while preserving the physical Jacobian for an equivalent physical target, subject to the documented representation.
- Singular configurations are reported clearly without blocking ordinary FK or pose inspection.
- Jacobian analysis does not modify joint values, accepted DH state, or validation state.

## UI-6 — Viewport, camera, and selection presentation

### Tasks

- Fit the robot after loading while respecting subsequent user camera adjustments.
- Provide separate Fit Robot and Fit Selection actions.
- Add Front, Side, Top, and Isometric camera presets with documented coordinate conventions.
- Group geometry/frame/layer toggles in a compact viewport toolbar.
- Offer a subtle ground grid and restrained lighting that preserve body detail in both themes.
- Scale displayed frame axes relative to robot size, with user override.
- Use consistent selection outlines across the tree, robot geometry, and DH frames.
- Avoid automatic camera jumps during ordinary joint motion or property editing.
- Provide sensible fit behavior for empty selections, hidden geometry, and missing-mesh fallback.

### Acceptance criteria

- Fit actions frame the intended geometry without clipping and change only the camera.
- Joint positions, DH values, and accepted session state are unchanged by view operations.
- Both small and large robots remain inspectable with appropriately sized axes.
- Robot geometry and selection remain legible in light and dark themes.

## UI-7 — Menus, visual consistency, accessibility, and status

### Tasks

- Group project actions in File; editing actions in Edit; layout, camera, and theme preferences in View.
- Move the theme selector out of the primary workflow toolbar.
- Use consistent typography, spacing, button heights, icon style, and focus indication.
- Use one primary selection accent; reserve warning/error colors for meaningful conditions.
- Provide text/icons as well as color to communicate state.
- Replace long status-bar paths with a concise project/file label; expose the full path on hover and through Copy Path/Open Folder.
- Move Include Local Assets into the relevant save/export dialog.
- Separate ready/progress messages, geometry availability, active-chain context, and validation status.
- Clearly mark validation as stale after relevant changes.
- Support keyboard navigation and readable focus states.
- Ensure long names, paths, and translated/scaled text do not obscure actions.

### Acceptance criteria

- Users can locate open/save, tool switching, camera controls, and validation without searching crowded panels.
- Both themes expose selected, disabled, focused, warning, and error states clearly.
- Status messages do not imply validation applies to a changed model.
- File paths remain accessible without occupying most of the status bar.

## UI-8 — Integrated verification and Windows delivery

### Tasks

- Review the implementation against every acceptance criterion above.
- Run relevant existing regression checks and add targeted tests for unit conversion, precision preservation, selection mapping, state handling, and persistence where behavior changes.
- Verify a mesh-backed serial robot and a robot with a different joint structure, including a prismatic joint.
- Use a branched fixture for whole-robot versus active-chain UI checks; its presence is not evidence of whole-tree DH support.
- Exercise missing assets, long names/paths, invalid numerical input, narrow panels, and empty selections.
- Check at least 1366×768 and 1920×1080 layouts, and Windows scaling at 100%, 125%, and 150% where available. Record actual combinations tested and any supported minimum size.
- Visually inspect light/dark themes and capture before/after screenshots of the same robot and pose.
- Verify save/reload and appearance/layout preferences without corrupting existing session formats.
- Rebuild the Windows distribution from the tested source revision and verify the integrated UI in the packaged executable.
- Exercise external robot assets from a different working directory. Record any separate clean-machine verification that remains unperformed.
- Update user documentation, screenshots, shortcuts, and known limitations.

### Completion record

For each milestone, record changed files, source revision, automated checks, manual UI observations, and remaining issues. Do not mark a stage complete from implementation alone.

The final delivery must identify the executable location, tested source revision, environment, screenshots, test results, and outstanding acceptance items. A Python-only change or an unchanged old executable does not complete UI delivery.

## Boundaries

- Implementation was explicitly requested by the user on 2026-09-25. Work remains within Stages 0–23.
- UI changes must not silently alter DH conventions, joint signs, limits, base/tool transforms, or source geometry.
- Existing and proposed mesh management, axis flips, FK calculations, and portable-project features retain their separate functional acceptance requirements. A redesigned control does not establish that its underlying feature is complete.
- No dynamics, control, simulation, hardware integration, or removed later-stage work is added by this plan.


## Implementation record — 2026-09-25

This is an incremental acceptance record, not a claim that every optional refinement above is finished.

| Milestone | Implemented and checked | Remaining acceptance / refinement |
|---|---|---|
| UI-1 | Maximized startup, hierarchy left, Model & Appearance on right, resizable/collapsible bottom DH/FK/Validation tabs, saved split sizes, reset layout | Tool choice and collapsed-result state are not yet persisted |
| UI-2 | SI storage retained; joint/DH/FK rad/deg and m/mm display; joint/table precision setting; full-precision tooltips and JSON; negative zero normalization; residual scientific notation | Source geometry fields and symbolic displays retain their existing explicit units/precision; global precision does not override every existing text report |
| UI-3 | Right-aligned unit-labelled DH cells, full precision tooltip, changed-value bold text, explicit pending context, sign column opt-in, copy displayed TSV, synchronized selected row | Identifier columns are not frozen; dedicated new DH CSV export not added (existing model/session exports remain) |
| UI-4 | Compact per-joint rows, joint type/limits/viewing range, slider and numeric entry, individual reset with clamping, unit-aware increment | Separate user-defined step size per joint remains a refinement |
| UI-5 | Click physical body selects link and scrolls tree; golden moving bounding box; matching DH row; background deselection retained; active-chain bold plus text; context focus/isolate/inspect; isolation restores previous appearance | More differentiated icons/warning badges remain; automatic selection does not open the inspector tool |
| UI-5A | FK shares model selection with analysis: URDF/automatic/committed/preview; unavailable preview clears results; bottom transform grid; SI JSON with source/model/preview metadata | Dedicated searchable physical-frame selectors and extra orientation sub-tables remain |
| UI-5B | Full geometric J, Jv/Jw, rates, twist, normalized SVD/rank/condition and full-precision SI export; moving-reference and named-point support; wide dialog; no pose mutation | Analytical RPY Jacobian intentionally unsupported; singularity diagnostics depend on stated L and fixed relative threshold 1e-9 |
| UI-6 | Fit robot/selection, front XZ, side YZ, top XY, isometric; optional non-pickable XY ground grid; lightkit; moving selection outline | Automatic robot-size-dependent frame-axis sizing not added; existing manual Frames override remains |
| UI-7 | File/Edit/View/Tools/Help groups; theme removed from toolbar; short source status, path tooltip/copy; asset checkbox in Save As; stale-validation text; consistent tree selection accent | Native assistive-technology and multi-monitor validation remain; portable ZIP always includes local assets by design |
| UI-8 | Automated core, mypy and native evidence; Atlas mesh, SCARA prism, repeated robot switching; native Qt 1.0/1.25/1.5 scale-factor exercises; packaged tests recorded separately | Qt scale-factor checks are not certification on three physical Windows display configurations; separate clean-PC test remains outstanding |

Changed code: `ui/workspace.py`, `ui/analysis_panel.py`, `jacobian_analysis.py`, `ui/workspace_check.py`, `ui/desktop.py`, `ui/studio.py`, `ui/fk_panel.py`, `ui/number_display.py`, `ui/project_panel.py`, `ui/robot_switch.py`.

Verification: 375 pytest tests (including 11 new relative-Jacobian cases); mypy 65 modules; native workspace, numeric precision, appearance/project persistence, switching, presentation/Atlas checks. See release record for final source revision, frozen executable tests and hashes.

No later-stage dynamics or hardware code restored. Screenshots are verification captures of the application, not evidence for unsupported research or production claims.
