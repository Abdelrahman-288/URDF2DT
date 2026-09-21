# Stage 23 desktop studio verification — 21 September 2026

Built source: `2ac5405a0c0e69d3236b60676921eb3f0785842c`, clean checkout of
`codex/stage23-desktop-improvements`. This record documents that executable;
later documentation-only commits do not change its embedded source revision.
Scope remains Stages 0–23. Removed dynamics/control/hardware stages were not restored.

## Delivered functionality

| Increment | Integrated behavior | Evidence |
|---|---|---|
| Body and asset handling | Multiple visuals/collisions, primitives, mesh origins/units/materials, assignment/replacement, caching | Frozen mesh-backed UR5: 15 geometry elements; SCARA: 6. Actor reuse, missing-file location, mm scale, and corrupt-COLLADA isolation passed. |
| Structure and inspector | Hierarchical search/filter, multi-selection, visibility, aliases, source/joint/frame properties, appearance undo, file actions | Native callbacks, per-element status, unchanged accepted DH during appearance edits; reviewed light/dark captures. |
| DH conventions | Proper-rotation X/Z flips, adjacent compensation and joint signs, preview/accept/reject, history/restore | Every-frame repeated/combined flips for UR5 and revolute/prismatic SCARA; undo/redo, single-frame restore, archive reload and sampled FK passed. |
| FK inspection | Live SI or degree input, physical reference/target alignment, matrix/rotation/quaternion/RPY, named attached frames and pose library | Analytical offset/gimbal-lock tests; frozen calculator and named-frame persistence passed. |
| Projects | Save/Save As/ZIP, source preservation, relative copied assets, appearance/mappings/session state, recovery/checkpoints | Frozen relocation and resave/reload passed; malformed project rejected before replacing active application. Duplicate asset/material groups tested. |
| Application quality | Resizable/scrollable panels, themes, unsaved-change handling, geometry progress, reports, measurements, batch checks | 1500×940 and 1280×800 native captures reviewed. Existing session/collision verification retained. |
| Windows distribution | EXE with embedded runtime, dependency notices, example robots and guide | Extracted ZIP, 1,575 manifest file hashes, legacy and new frozen suites passed outside the repository. |

## Test evidence

- Local complete regression suite: **329 passed**, 58.40 seconds.
- Mypy: **48 source files**, no issues.
- [CI for the exact packaged source](https://github.com/Abdelrahman-288/URDF2DT/actions/runs/35569813934):
  Ubuntu/Python 3.10, Ubuntu/Python 3.12, Windows/Python 3.12 all passed.
- `--verify-package` exited zero from the extracted distribution with Python/Git
  absent from PATH. Session export/reload, FK, themes, actor reuse and collision
  display passed for both bundled robots.
- `--verify-studio` exited zero from the same extracted EXE and verified its
  embedded commit. External doctor UR5/STL assets and the bundled SCARA were
  loaded, edited, saved, relocated, reloaded and resaved.
- Frozen mean pose-update measurements over ten updates, including event handling:
  SCARA **26.01 ms**, mesh-backed UR5 **55.58 ms**. These are development-host
  observations, not a frame-rate guarantee or a large-model stress benchmark.

ZIP: `URDF2DT-v1.0-Windows.zip`, **200,550,154 bytes**.

SHA-256:

```text
9d3afa8ac80e35f0dcb552fb23e0dc50dda173ac50e4475720a348c737415ad8
```

Local delivery is in the project root's `Windows Application` directory. Open
`URDF2DT.exe`; keep `_internal` beside it. The ZIP is the application distribution
to share. A user's portable robot-project ZIP is separate. The previous application
is retained under `outputs/tmp/windows-application-before-studio-4c9f117`.

Machine-readable reports beside the delivered application:
`delivery-verification.json` and `studio-verification.json`. Original captures and
relocated test projects are retained in Downloads/URDF2DT-Desktop-Studio-2ac5405,
under `verification` and `studio-verification`. These contain local test paths;
the test assets are not published as newly licensed application content.

## Remaining verification and limits

This is a verified development-host research candidate, not an industrially
certified or production-ready application. A separate clean Windows PC/VM test,
broader GPU/high-DPI/accessibility coverage, code signing, and formal technical
advisor sign-off remain pending. Python/Git removal from PATH is not a clean-PC test.

Frozen body acceptance used STL and primitives. Other supported formats and texture
variants have not received the same comprehensive native acceptance coverage.
Complex shaders are not reproduced; supported OBJ/DAE materials may be sampled to
vertex colors. URDF textures require UV coordinates. The doctor UR5's bundled STL
substitutes for unavailable DAE are labeled proxies, not newly inferred geometry.

Individual mesh decoding and global FK checks remain synchronous, so unusually
large files can pause the UI despite loading progress. Project recovery can depend
on external files; use portable ZIP for relocation. Undo stacks are in-memory,
while accepted conventions/audit and named poses/frames persist. Full-tree body
display does not broaden mathematical certification: inactive branches stay at zero,
and DH/FK validation applies to the selected supported serial path.

See [the usage guide and format contract](../desktop_improvements.md) for precise
flip transformations, persistence schemas and supported asset dependencies.
