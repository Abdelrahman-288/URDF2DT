# Stage 23 imported robot bodies — 21 September 2026

The subsequent [live appearance delivery](stage23-live-appearance.md) retains these
body fixes and updates the opacity/color controls and Windows executable.

Built source: `8b73e85c95a00f93a1c38f3a15e355ea75aa7c86`, clean checkout of
`codex/stage23-desktop-improvements`. Scope remains Stages 0–23. This supersedes
the earlier desktop delivery for imported mesh discovery; existing studio features
remain in place. Later documentation-only commits do not change the executable.

## Problem and delivered behavior

The screenshot's ABB IRB140 had mesh files, but its URDF used an original ROS
package name while the imported directory had been renamed. The resolver searched
the wrong locations and the renderer displayed its diagnostic skeleton. Discovery
now supports sibling meshes, neighboring packages, generated files, extracted
monorepo prefixes and unambiguous visual/collision shorthand. Explicit mappings
take priority and ambiguous model variants require manual selection. No mesh is
invented or downloaded, and the user's source files remain unchanged.

Robot Structure now shows body completeness. Shared inline materials retain the
ABB orange color. Namespaced Drake acceleration metadata is preserved with a
kinematic-validator warning and is not modeled. Other unknown limit attributes
remain rejected. This adds no dynamics functionality.

## Verification

- Full local suite: **341 passed**, 67.77 seconds; mypy passed for 49 source files.
- [GitHub CI](https://github.com/Abdelrahman-288/URDF2DT/actions/runs/35603699772)
  passed Ubuntu Python 3.10/3.12 and Windows Python 3.12.
- Native source and frozen `--verify-library` passed on ABB IRB140 (7 visual
  elements), ABB IRB6700 (9), FANUC LR Mate 200iD (7), KUKA KR120 (7) and iiwa14
  (15). All their visual/collision records loaded. The check moved a joint,
  verified actor reuse and unchanged accepted DH state, then saved and reopened
  an ABB project with copied meshes. Body screenshots were inspected, including
  the frozen ABB capture. STL, COLLADA and OBJ were represented.
- Frozen `--verify-studio` passed on SCARA, including appearance, asset replacement,
  malformed asset isolation, frame edits/FK and portable project behavior.
- Frozen `--verify-package` passed after extraction outside the repository with
  Python/Git removed from PATH. All **1,575** manifest files matched checksums.
- A separate read-only path audit found all mesh references for 288 of 306 local
  URDF files: 7,462 of 7,791 references resolved. Twelve files had unresolved
  references; six contained no meshes. This audit is file discovery, not rendering
  or mathematical certification of the whole collection.

The delivered ZIP has 200,556,217 bytes and SHA-256:

```text
327b73e26795ceef03b74070c1c121da9af62cdf597e42efb5fa4404f8d20d95
```

## Local delivery and use

Launch `Windows Application/URDF2DT.exe` in the project root. Keep `_internal`
beside it. Use **Open URDF** and select `robots/abb_irb140/urdf/irb140.urdf`, then
move the joint sliders. The status should read **7/7 elements loaded**. To share
the application, send `Windows Application/URDF2DT-v1.0-Windows.zip`. External robot
packages are separate assets; keep their folder structure or save a portable
project with assets when sharing a model.

The previous application is retained under
`outputs/tmp/windows-application-before-mesh-discovery-2ac5405`.
Delivery, library and studio JSON reports are beside the new root executable.
Original reports, screenshots and test projects are retained in
`C:/Users/abdel/Downloads/URDF2DT-Mesh-Discovery-8b73e85`.
The local collection and captures are not republished as licensed robot content.

## Remaining limitations

Missing or ambiguous dependencies still require the actual files or a manual
mapping. Some robots fall outside the supported kinematic subset. Full-tree body
display does not certify inactive branches. Complex materials and shaders may
differ from their authoring tools; large native mesh decodes can pause the UI.
Clean-PC testing, broader graphics/DPI/accessibility coverage and code signing
remain pending. This is a development-host verified research candidate.
