# Stage 16: Invariant and regression testing

The seeded sequence suite exercises both the six-axis UR5 and the mixed-joint
four-axis SCARA through the real editor and persistence APIs. It supplements
the earlier single-edit tests and Stage 14 independent parameter sweeps with
accumulating edits, rejection, partial restoration, and repeated acceptance.

## Coverage

Three seeds (7, 42, 113), two robots, and twelve proposal steps per sequence
produce 72 proposal steps: 54 accepted edits and 18 deliberate rejections.
Single-frame restoration occurs eighteen times in total. Each sequence also
performs a validated archive round trip and a whole-session restore.

After every step the tests check:

- Automatic model object identity and serialized contents remain unchanged.
- Proposals and rejected previews preserve committed geometry.
- Accepted upstream rows remain exactly unchanged, and downstream acceptance
  is invalidated after a changed edit.
- World frames other than the edited frame remain unchanged within numerical
  tolerance during compensated editing.
- All zero-pose frames satisfy local rigidity, joint-line preservation, and
  common-normal checks relative to the automatic baseline.
- Rotation matrices are orthonormal with determinant +1 at ten seeded poses.
- Edited DH FK agrees with URDF FK at those same poses.
- Single-frame restoration restores its baseline world frame, and whole-session
  restore recovers the exact baseline model and initial acceptance states.
- Save/reload preserves the entire editor state and retains FK equivalence.

Separate tests verify identical sampling for a fixed seed, a changed nonzero
sample sequence for a different seed, and bounded rotary/prismatic joint limits.
The configuration count includes the zero pose, consistent with the existing
validator's sampling contract.

## Reproduce

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_invariant_sequences.py -q
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m mypy
.\.venv\Scripts\python.exe -m pip check
```

The eight added tests use fixed seeds for replay. They do not claim exhaustive
coverage of all legal edit sequences. Matrix comparisons use absolute tolerances
with relative tolerance disabled; sampled FK remains numerical evidence rather
than a continuous-space proof. Local rule names are descriptive while the external
report's R1–R3 mapping remains provisional.

No production algorithm changes were required by these regressions. The next stage
is Stage 17: continuous integration to run the suite automatically on GitHub.

## Recorded verification

All 300 tests passed, including the eight new invariant tests. Mypy checked 36
package files without errors, and pip check found no broken requirements.
