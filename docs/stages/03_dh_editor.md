# Stage 3: contracts, configuration, and immutable values

## Decisions recorded before implementation

The user confirmed on 2026-09-13 that this is a new repository with no existing
parser or automatic DH solver. On the same date they authorized proceeding with
Stage 3 despite the outstanding references. The contracts below are new local
interfaces, not interfaces recovered from an existing parent project.

The official parent module numbering remains unverified until `main.pdf` is
provided. Here, parser and solver are named by responsibility. The plan's
`02_dh_editor.md` reference is an inconsistent filename; this record is canonical.

Thresholds come from section 10 of the supplied plan and remain **provisional**.
No advisor approval is claimed. Classification and FK research validation must
reconcile these values with the missing technical report before claiming results.

## Local interface contract

- Stage 3 originally proposed `URDFParser.parse(path: Path, config: EditorConfig)`.
  Stage 4 refined this to `parse(source: ValidatedURDF, config: EditorConfig)
  -> KinematicChain` to consume the exact validated snapshot without reopening
  a changed file. Stage 5 implements conversion into the kinematic chain.
- `DHSolver.solve(chain: KinematicChain, config: EditorConfig) -> DHModel` returns
  a Standard-DH baseline with one row per movable joint, in chain order.
- Protocols live in `urdf2dt/interfaces.py`; these are contracts, not implementations.
- `KinematicChain` retains fixed joints and their transforms in base-to-tip order.
  Joint origins map the child frame at zero displacement into the parent frame;
  axes are unit vectors in the joint frame. Movable joint names define q order.
- `DHModel` explicitly carries base/tool transforms and joint signs. Using column
  vectors, FK will be `base_transform @ product(A_i(q_i)) @ tool_transform`, where
  `A_i = Rz(theta_i) @ Tz(d_i) @ Tx(a_i) @ Rx(alpha_i)`.
- Revolute/continuous: `theta_i = theta_offset + joint_sign*q_i`; prismatic:
  `d_i = d + joint_sign*q_i`, with theta fixed at theta_offset. Lengths are meters,
  angles radians. DH rows/frame indices are one-based; q tuple positions zero-based.
- Joint limits are required for revolute/prismatic joints; continuous and fixed
  joints have no finite position limits in this contract.
- Numeric arrays are copied into nested tuples. Frozen dataclasses alone would
  not protect a NumPy array from mutation. No UI libraries are imported by core.
- `FrameState` is a status enum; `EditorState` is an immutable snapshot. Transitions,
  geometry classification, local validation and FK algorithms belong to later stages.
- `AxisCase` has descriptive categories only. A/B1/B2 mappings are intentionally
  undefined until the report is available.

## Temporary reference

`tests/fixtures/ur5_automatic_dh.py` contains nominal classic UR5 dimensions from
[Universal Robots](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics),
retrieved 2026-09-13. It is a temporary DH-only fixture, not solver output and not
UR5e data. Joint names are synthetic; zero offsets, positive signs and identity
base/tool transforms are fixture conventions, not verified URDF alignment.
The missing MATLAB files and URDF must be compared before replacing the fixture
with actual automatic output. No URDF/FK equivalence is claimed.

## Provisional output schema

`schemas/dh-model-0.1.schema.json` defines a draft model envelope, independent of
the eventual validated-session schema 1.0. It includes provenance, parameters,
frame transforms, and a configuration snapshot. It deliberately carries no PASS
flag: Stage 3 cannot produce a validated session. Serialization and semantic
reload validation remain Stage 12 work; JSON Schema checks structural shape only.

## Verification

- `python -m pytest -q`: **57 passed**, no skips or xfails.
- `python -m mypy`: **no issues in 11 production source files**, targeting Python 3.10.
- `python -m pip check`: **no broken requirements**.
- Editable installation with `.[dev]` succeeded. An isolated subprocess outside
  the repository imported the installed core with UI dependencies actively blocked.
- Tests cover copied NumPy/list inputs, frozen nested records, immutable baseline
  replacement, rigid-transform validity, serial joint order, fixed-joint retention,
  strict YAML fields/duplicates/security, deterministic configuration snapshots,
  measurement dimensions/tolerances, and draft-schema positive/negative examples.
- Runtime verification used Python 3.12.14 on Windows. Other Python versions and
  platforms have not yet been exercised. Targeting 3.10 in mypy is not runtime CI.

## Status and next gate

Stage 3 implementation is complete with the user-authorized provisional inputs.
The original plan's external reference-confirmation gate remains open: thresholds,
parent module numbering, MATLAB comparison and URDF alignment are unverified.
No classification or FK-equivalence result was produced by this stage.

Branch: `feature/stage-03-core-types`, based on Stage 2 commit `4c7c067`.
Next implementation stage: URDF input and structural validation (Stage 4).
