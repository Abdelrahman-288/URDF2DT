# Stage 24 numerical evidence

Model source commit: `ea26ae18d1765b758bc2e2ed2c51d14f6edd1f95`.
Both report runs recorded a clean checkout before evidence was added.
Environment: Windows x64, Python 3.12.14; pinned dependencies are in
`requirements/windows-py312-dynamics.lock`.

| Metric | UR5 | Nominal SCARA |
| --- | ---: | ---: |
| Samples / seed | 64 / 2401 | 64 / 2401 |
| Maximum inverse effort error | 3.56e-14 N m | 2.74e-10 N m (revolute); 0 N (prismatic) |
| Maximum forward acceleration error | 6.73e-14 rad/s² | 7.11e-9 rad/s² (revolute); <2e-15 m/s² (prismatic) |
| Maximum mass-matrix entry difference | 6.67e-15 | 1.74e-10 |
| Maximum link-pose matrix difference | 8.89e-16 | 6.67e-16 |
| Outcome | PASS | PASS |

Per-joint errors, units, thresholds, every sampled state, model versions and timing
are retained in each `validation.json`. The mass-matrix entries have joint-dependent
units; the table is a numerical comparison, not a mixed-unit physical norm.
SCARA's residuals reflect the reference engine's tensor diagonalization precision.
The reports explicitly describe the inertial-rotation normalization in the adapter.

Each `dynamic_model.json` contains the exact source snapshot and inertial/config
parameters. It can be reloaded without the original URDF path. The schema hash is
an integrity check, not an authentication signature. These are nominal models;
none of these files establishes accuracy for physical hardware.

The fresh installed environment passed 345 tests with no skips, mypy checked
48 source files, and pip reported no broken requirements. Tests include the
analytical pendulum, two-link coupled arm and prismatic/fixed-tool system, plus
physical-data rejection, independent-reference and persistence regressions.

## Repeat

From this branch's checkout and its activated Python 3.12 environment:

```powershell
python -m urdf2dt.dynamics robots/dynamics/ur5_dynamics.urdf --output outputs/tmp/repeat-ur5
python -m urdf2dt.dynamics robots/dynamics/scara_dynamics.urdf --output outputs/tmp/repeat-scara
python -m pytest -q
python -m mypy
```

Choose new output directories for each run. Timing and Git metadata change on
repetition; seeded states and numerical results should agree within the recorded
engineering thresholds. Advisor agreement on experiment acceptance thresholds,
Stage 23 clean-PC acceptance, identification and hardware validation remain open.
