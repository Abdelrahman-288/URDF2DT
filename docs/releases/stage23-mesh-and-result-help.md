# Mesh troubleshooting and kinematics explanations

## Locate and preview mesh replacements

Open Edit → Mesh troubleshooting, or select a geometry element and choose
Model & Appearance → Geometry → Missing file / search details.

The diagnostics list identifies each link and visual/collision element, status,
original URI, effective reference, source URDF, resolved file and actual ordered
file candidates checked by the resolver. Missing and unsupported elements appear
first. Copy diagnostics copies the complete report, including local paths.

Locate file stages a replacement. Locate folder tries the reference in a chosen
package folder; a moved absolute file can be proposed by its exact filename in
the explicitly selected folder. Failed folder attempts display their search trace.
Neither operation automatically accepts a replacement or edits the source URDF.

Inspect the preview on the robot, including its dimensions, units and orientation.
Use Apply preview to enter the property undo history or Cancel preview to restore
the previous geometry. Pending geometry previews cannot be saved as projects.
Invalid or unreadable replacement files preserve the previous geometry. Replacements
affect one geometry element, not every mesh in a package, and do not change DH/FK.

The existing Mesh package directory action remains a separate bulk mapping action.
This update does not add automatic online mesh retrieval or CAD repair. Matching
names are not proof that a replacement is the correct physical part.

## Understand FK and Jacobian results

Numerical FK now explains the target-frame origin, reference axes, live display
units and transform blocks beside its output. It distinguishes a pose at current
joint values from the symbolic T(q) formula and full-precision SI exports.

Jacobian analysis explains its target point and reference, moving-reference
semantics, linear/angular rows, source joint-column order, joint-rate units and
output velocity units. Angular velocity is explicitly distinguished from RPY rates.
Matrix-entry units depend on row and joint type.

Symbolic FK/Jacobian scope now includes a q-symbol-to-source-joint key, SI
coordinate units and the distinction between formulas and numerical evaluation.
The symbolic tools use active-chain endpoints independently of numerical FK frame
selectors. Geometry constants retain their stored precision; no solver is changed.

## Verification

`--verify-mesh-help NEW_FOLDER` exercises actual mesh preview dimensions, cancel,
apply/undo, pending-save rejection, invalid replacement preservation, diagnostics
and result explanation UI. Resolver tests cover missing candidates, explicit mapping
priority, invalid package URIs and moved-file folder proposals.

## Delivery — 26 September 2026

Build `5cd1073`: 381 tests passed in 64.24 seconds; mypy passed for 72 source
files. Native and frozen mesh/result checks passed with inspected screenshots.
Frozen workspace/Jacobian regression checks passed. Evidence is retained in
`outputs/tmp/mesh-results-native-final`, `outputs/tmp/mesh-results-frozen` and
`outputs/tmp/mesh-results-workspace`.
The installed executable also passed the mesh/result check at its final location
(`outputs/tmp/mesh-results-installed`).

Installed in `Windows Application`; the previous version is backed up at
`outputs/tmp/application-before-mesh-results-5cd1073`. The PDF's new-behavior
addendum is `User Manual/Mesh and Results Update.txt`. Preserved unrelated local
work is honestly reflected in the non-clean build metadata. Clean-PC verification
remains a separate pending check.
