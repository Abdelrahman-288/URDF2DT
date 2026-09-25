# Atlas body, matrix readability and tool navigation delivery

Delivered from clean source d76193ec04b7097dac5714dd6f80ec7bfc6913a4.
The earlier robot-switching, Open Example removal and background deselection
changes are included (see stage23-robot-switching.md).

Atlas rendering: the supplied URDF references unavailable OBJ assets, while the
same mesh directory contains same-stem STL files. Exact OBJ files retain priority;
unambiguous same-directory/stem STL substitutes are identified in the inspector.
No source file was rewritten. All 50 visual elements loaded: 30 mesh instances
from 24 distinct STL files, plus 20 primitives. Pose motion was verified. Atlas
remains an external test dataset, not a bundled or relicensed application asset.

Navigation: a Tool selector beside Fit view selects the existing right panel.
The five tools no longer compete for space in a horizontal tab strip. The right
panel can be widened. Symbolic matrices use labelled cells with clickable M
entries, browsable definitions and original full-precision text. Floating literals
are shortened only in an explicitly approximate readable preview; tiny nonzero
terms use scientific notation. Original expressions, calculations, clipboard and
exports retain precision. Numerical Jacobian has labelled three-decimal cells and
full-precision JSON export. The full matrix dialog supports the selected view.

Evidence:

- 364 tests passed in 55.92 seconds; mypy passed for 61 source files.
- Resolver tests cover exact STL substitutes, original OBJ precedence and missing
  assets. Presentation tests retain exact pi terms, tiny coefficients and original
  export text while supplying structured layout data.
- Native presentation, repeated switching, symbolic FK and Jacobian checks passed.
- Frozen package, presentation (including external Atlas), switching, studio,
  numbers, symbolic FK and Jacobian checks all passed outside the source checkout
  with Python and Git removed from PATH.
- Source 1280x800 matrix views and source/frozen Atlas captures were inspected.
- All 1617 extracted manifest entries matched their checksums.

Installed: project-root Windows Application/URDF2DT.exe.
Backup: outputs/tmp/windows-before-atlas-matrix-tools-d76193e.
Reports/captures: C:/Users/abdel/Downloads/URDF2DT-Atlas-Matrix-Tools-d76193e.
ZIP bytes: 208909255. SHA-256:
9a906218af7261bc796c6a9365c21a83397640181c48328c5448820e6fb0dc87

Limits: an STL substitute supplies geometry, not unavailable OBJ materials or
textures. Atlas motion follows the selected chain; other branches remain at zero.
Large matrices still support scrolling and an expanded view. Compact symbolic
previews are approximate; use full-precision exports for computation. The exact
reported universal third-open failure was not independently reproduced; repeated
interactive loads and failure recovery passed. Clean-PC/GPU-wide acceptance remains
pending. No removed Stage 24+ functionality was restored.
