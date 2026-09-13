# Stage 4: URDF input and structural validation

## Implemented behavior

`validate_urdf(path_or_snapshot)` returns a `URDFValidationResult` with stable
issue codes and user-facing messages. Invalid inputs return no document.
`result.require_valid()` returns the checked `ValidatedURDF` or raises
`URDFValidationError` with the same structured issues.

`URDFInput.from_path()` adapts a file picker path; `None`/empty string is a cancelled
selection. `URDFInput.from_upload(name, content)` adapts a notebook upload without
writing it to disk. Filenames must end in `.urdf` (case-insensitive). Bytes are
owned by an immutable snapshot with a SHA-256 digest and optional source path.

XML parsing always uses `defusedxml` with DTDs, entities and external references
forbidden. Neither Xacro nor XInclude is executed. Relative mesh paths, `file://`,
`package://`, and HTTP assets in visual content are never resolved. This is a
kinematic structural validator, not a complete visual/dynamics URDF schema check.

InputPolicy controls resource bounds independently of geometric tolerances:
5 MiB input, 20,000 XML elements, depth 100 by default. File reads are bounded to
max_bytes + 1. Element/depth limits are checked after parsing the byte-bounded XML.
These are implementation safety limits, not advisor-confirmed research thresholds.

## Accepted subset

- Expanded URDF format 1.0 with an unnamespaced `<robot>` and nonempty name.
- Named links and joints without duplicate names (within each category).
- A single connected acyclic serial path, with exactly one base and one tip.
  XML declaration order does not determine kinematic order.
- Fixed, revolute, continuous and prismatic joints; at least one movable joint.
- Finite origin XYZ/RPY triples; absent origin/attributes default to zero.
- Axis defaults to `(1, 0, 0)` when absent, as in the
  [ROS URDF parser](https://docs.ros.org/en/diamondback/api/urdf/html/joint_8cpp_source.html).
  Present axes must be finite nonzero triples and are normalized. This is not
  the DH axis convention; Stage 5 must preserve that distinction.
- Revolute/prismatic joints require explicit finite lower/upper, effort and
  velocity limits. Lower must not exceed upper; effort/velocity are nonnegative.
  Continuous joints may omit limits; if provided, effort/velocity are required,
  and position limits are ignored with a warning. Fixed position bounds likewise
  do not become JointLimit values. This stricter v1 policy avoids silent bounded
  joint limits when a URDF omits lower/upper.
- Top-level material, transmission, Gazebo and ros2_control content is recognized
  but not executed. Control/simulation extensions yield an ignored-content warning.

Branches (even fixed auxiliary frames), multiple parents, cycles, disconnected
components, mimic coupling, planar/floating joints and unexpanded Xacro fail with
specific codes. Unknown top-level/joint elements and kinematic-field attributes
are rejected rather than silently accepting misspelled origins or axes.

## Interface refinement from Stage 3

`URDFParser.parse(source: ValidatedURDF, config: EditorConfig) -> KinematicChain`
now consumes the validated byte snapshot and normalized fields, rather than a path.
Reopening a file would allow its contents to change after validation. There was no
existing implementation to migrate. The full parser still belongs to Stage 5;
Stage 4 does not calculate rotation matrices, generate DH, or run FK.

## Fixtures and research limits

See `robots/ur5/README.md` for pinned upstream source, licensing, checksums and
the exact transformation into a serial fixture. The unchanged upstream UR5 is a
branch-rejection fixture. The explicitly derived serial description is accepted.
Neither is represented as the still-missing MATLAB `universalUR5.urdf`.

`robots/examples/mixed_joints.urdf` exercises all four supported joint types.
`tests/fixtures/invalid_urdf/` includes malformed, branched, disconnected, cyclic,
missing-link, XXE and entity-expansion inputs. Security tests also attempt local
canary-file and network entities with external access blocked, including UTF-16.

## Headless entry point

```powershell
python -m urdf2dt.parser robots/ur5/ur5_serial.urdf
python -m urdf2dt.parser robots/ur5/ur5_serial.urdf --json
```

Exit 0 means structurally accepted, 1 rejected, 2 invalid CLI usage. JSON includes
issue codes and the source digest, but is not the final validated-DH export schema.

## Verification

- `python -m pytest -q`: **127 passed**, no skips or xfails.
- `python -m mypy`: **no issues in 14 production source files**.
- `python -m pip check`: **no broken requirements**.
- Installed CLI accepts the serial UR5 fixture: 9 links, 6 movable joints,
  `world -> tool0`; subprocess tests verify success/failure exit codes and JSON.
- Security tests reject DTDs, XXE, internal entities, external DTDs and UTF-16 DTDs.
- Graph tests cover cycles, branches, disconnected components, converging parents,
  reordered XML, and a 1,100-joint path without recursive graph traversal.
- Tests pin both UR5 checksums and compare every retained joint attribute against
  the upstream description. `.gitattributes` preserves these bytes across checkout.
- Runtime: Python 3.12.14 on Windows; cross-platform runtime CI remains future work.

## Status

Stage 4 implementation and local verification are complete on
`feature/stage-04-urdf-input`, based on Stage 3 commit `f7b3458`.
The exact MATLAB fixture and FK/reference comparison are still outstanding;
the public, explicitly prepared serial fixture is the acceptance input for now.
Next: Stage 5 parser and automatic DH integration.
