# Stage 18: Code quality and interface review

## Review outcome

The headless parser, DH solver, immutable editor session, sampled FK validation,
JSON persistence and optional UI layers retain their established contracts.
The integration branch is ready for main after its hosted checks pass.

- Moved rooted-tree selection and local asset resolution to
  `urdf2dt.parser.robot_document`. The old UI import remains a compatibility alias.
  Parser imports do not load Qt or the UI package.
- Reject invalid chain indices and invalid raw proposal types explicitly.
- Validate complete desktop poses before changing any joint; valid poses still
  clamp to the control limits. Invalid lengths, booleans and non-finite inputs
  cannot leave a partially changed pose.
- Added missing public docstrings, including SI units, index conventions, state
  transitions and local versus global validation responsibilities.
- Consolidated repeated representation and round-off guards without changing
  their values. Research geometry/FK tolerances remain configuration settings;
  display cutoffs remain local to rendering.
- Added logger warnings for caught desktop/notebook actions while retaining
  visible user feedback and the existing application logging configuration.
- Used the existing schema version constant consistently. Schema 1.0 fields,
  Standard-DH convention and existing archive decoding are unchanged.

## Architecture and compatibility audit

Production kinematics contain no UR5 link dimensions. Robot names and sample
paths in presentation/research entry points are examples; mesh fallback paths
are asset-resolution policy, not solver geometry. SCARA oracle constants belong
only to the independent fixture verification. Both native and notebook editors
continue to delegate geometry and acceptance to the domain session.

Existing public parser/solver and application entry points remain available.
The agreed type-checking baseline is the configuration in pyproject.toml,
including checking untyped function bodies; this is not a claim of strict typing
for every optional Qt/VTK object. Public functions/classes and public methods
were audited for docstrings. The full implementation diff was reviewed for
input handling, immutable state, numerical budgets, import boundaries and
persistence compatibility, with targeted regressions for the changes above.

## Verification

- Full local suite: 314 tests passed; 14 new interface regression cases.
- Mypy: no errors in 38 source files; pip check: no broken requirements.
- Existing Stage 15 archive passes schema validation and exact domain/JSON
  round-trip comparison.
- Native UR5 smoke: 30 pose updates reused actors; sampled FK passed;
  collision decomposition produced 56 parts.
- Native SCARA smoke: four joint controls, six mesh actors, the prismatic slider
  reached 0.18 m, accepted edits passed sampled FK, and actors were reused.
  Light and dark screenshots were generated; the dark view was visually reviewed.
- The SCARA analytic oracle's maximum matrix-element error was approximately
  7.8e-16 over 200 configurations; archive round-trip passed.

See [native verification records](../../outputs/review/stage18_native.json).
Hosted checks use the Stage 17 three-job Linux/Windows matrix and gate the
integration push to main. The README badge follows main after integration.

## Remaining limits and handoff

Sampled FK is not a continuous-space proof. Reference reconciliation and advisor
acceptance of tolerances remain pending. Archive reload deliberately rechecks
saved reports and may reject numerical differences across library/platform
versions; no universal archive portability or cryptographic audit claim is made.
The automated SCARA script's participant field is not a human-study result;
see the separate [recorded user observation](15_usability_observation.md).

Stage 19 is the final worked notebook, examples/ur5_full_pipeline.ipynb.
Existing user edits to examples/interactive_editor.ipynb and local output sessions
are excluded from this review commit.
