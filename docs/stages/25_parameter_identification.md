# Stage 25 — Dynamic Parameter Identification

## Scope and status

Implemented: source-bound inertial/friction identification, physical consistency,
identifiability and excitation diagnostics, approximate uncertainty, complete
trajectory holdout, versioned datasets/parameter archives, trusted manufacturer
mode, calibrated-current conversion and conditional motor identification.
UR5 and structurally different SCARA benchmarks use independent MuJoCo efforts.

This stage is a headless Python/API workflow. The Stage 23 Windows executable is
unchanged. These experiments test recovery from synthetic observations; they do
not establish hardware accuracy. Stage 30 must repeat calibration/evaluation with
physical measurements. Stage 26 will add motion-reference generation; these
identification excitation signals are not commands for a real robot.

## Reproduce

Use Python 3.12, from this branch's checkout:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev,research,identification,dynamics-reference]" ipywidgets
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m mypy
.\.venv\Scripts\python.exe -m urdf2dt.identification benchmark robots/dynamics/ur5_dynamics.urdf --output outputs/identification/ur5-1
.\.venv\Scripts\python.exe -m urdf2dt.identification benchmark robots/dynamics/scara_dynamics.urdf --output outputs/identification/scara-1
```

Each output directory must be new. It contains `training.json`, `held_out.json`,
`parameters.json` and `report.json`. All use a SHA-256 envelope, kind and schema
version 1.0, bounded reads and exclusive creation. SHA-256 detects accidental
changes; it is not an authenticity signature. Preserve all four files together.
`requirements/windows-py312-identification.lock` pins the tested headless Windows
environment; install it first, then install the editable package with `--no-deps`.
Core dataset/regressor contracts also run on Python 3.10; the pinned optional
CVXPY/MuJoCo extras and full benchmark are required in Python 3.12 CI.

## Model and parameter convention

For each moving link, including massive fixed descendants, estimate ten linear
parameters in its original URDF link frame:

`m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz`.

Here `h = m*c`, and `I` is about the link origin, not the center of mass. Units
are kg, kg*m and kg*m². The original inertial-frame rotation is incorporated when
converting nominal values. Two further coefficients per movable joint describe
`b*v + fc*sign(v)`; `sign(0)=0`, without stiction. Revolute effort is N*m and
prismatic effort is N. Archives retain explicit units and ordered names.

The energy regressor uses the original chain poses and exact CasADi derivatives:

```text
T = 0.5*m*vo.T*vo + vo.T*(omega cross R*h) + 0.5*omega.T*R*I*R.T*omega
V = -gravity.T*(m*position + R*h)
tau = d/dt(dT/dv) - dT/dq + dV/dq + friction = Y(q,v,a)*parameters
```

The kinematic snapshot hash and joint order must match every dataset and model.
Regressor construction requires validated kinematics but does not require usable
URDF inertias. A generic initialization is available: 1kg/link, COM at the link
origin, diagonal COM inertia 0.02kg*m², zero friction. This arbitrary initialization
is explicitly recorded. It is not inferred manufacturer data.

Stationary base inertias are unobservable and omitted in reconstructed models.
Explicit massless marker links may be fixed to zero using `massless_links`; this
choice is a prior and appears in every parameter archive. The UR5 benchmark uses
the source's explicitly massless `tool0`. Missing inertias are not silently
treated as massless. The SCARA fixed tool is massive and is estimated.

## Estimator, physical consistency and identifiability

Training data alone defines the least-squares objective. Independent generic
probe states (seed 2501) estimate attainable numerical rank. A training design
with lower rank is rejected. The relative SVD threshold defaults to 1e-8;
column scales derive from generic-probe column norms, and per-joint effort
weights are fixed beforehand (default 1 N*m or 1 N). Condition numbers, singular
values, scales and bases are saved. This is a numerical excitation check, not a
symbolic observability proof or assurance against all measurement-noise effects.

Reports define `beta = basis @ (parameters / parameter_scale)`. Beta describes
observable combinations. Nullity is reported separately; a fitted mass/COM/tensor
is one representative, not evidence that each raw parameter was independently
recovered. A 1e-6 penalty in the training nullspace selects a representative near
the recorded prior; it does not directly regularize observable combinations.

CVXPY 1.9.2 with Clarabel 0.11.1 solves the convex constrained least-squares problem.
For each body, the pseudo-inertia matrix is

```text
P = [[0.5*trace(I)*identity - I, h],
     [h.T,                       m]]
```

`P >= 1e-6*identity` is imposed after choosing reference mass 1kg and length 1m
(the displayed SI numeric values have those scales). This is a numerical
strict-interior margin, not a universal physical minimum. It ensures positive mass
and physically realizable COM inertia satisfying the principal-moment triangle
inequalities. Friction is nonnegative. Solver status must be `optimal`; tensors
are reconstructed and checked again by Stage 24's physical contracts, with
positive-definite mass matrices checked across training positions. Tiny negative
friction from solver roundoff may be clipped only below 1e-8 and is reported.
Parameters close to the margin may require an experiment-specific scaling policy.

The physical constraint follows [Wensing, Kim and Slotine, LRA 2018](https://arxiv.org/abs/1701.04395).
The code uses the affine matrix constraints documented by
[CVXPY](https://www.cvxpy.org/tutorial/constraints/index.html).
No CAD bounding-volume constraints or compliance/contact dynamics are assumed.

Approximate standard errors on beta use residual variance and retained singular
values. They assume exact q/v/a and independent, homoscedastic noise after effort
weighting. They are unconstrained linear approximations, not Bayesian intervals,
not raw-inertia confidence bounds and not valid treatment of arbitrary correlated
derivative errors. No uncertainty is claimed for unobservable individual values.

## Dataset contract and measured data

`IdentificationData` accepts synchronized arrays shaped `(samples, joints)` for
`q`, `velocity`, `acceleration` and joint-side `effort`, plus:

- `source_sha256`, ordered `joint_names`, per-joint `position_units` (`rad` or `m`).
- `trajectory`: a stable identifier per sample, retained across preprocessing.
- `time`: seconds, strictly increasing within each trajectory; at least two samples.
- `origin`: `simulated` or `measured`; `acquisition_id`: log/experiment identifier.
- `processing`: required description of derivative estimation, filtering,
  synchronization, noise/uncertainty assumptions and removed transients.
- Optional paired `current_ampere` and `CurrentCalibration`; both are retained.

Velocity and acceleration use the position unit per second and per second².
Do not relabel degrees, millimeters or motor-shaft positions as joint SI values.
Measured logs must be converted to these explicit contracts before fitting.
No automatic differentiation/filtering is performed: supply calibrated velocities
and accelerations, or document your filter/window/order, sampling interval,
edge trimming, delay and uncertainty. Separate complete trajectories before
filtering, so processing does not mix training and held-out observations.
Unknown derivative uncertainty must be reported as unknown, not zero.

For calibrated current, `effort = gain * (current_ampere - offset_ampere)`.
Each signed gain is N*m/A or N/A at the joint after transmission. The calibration
provenance must describe the motor/transmission assumptions; a motor torque
constant alone is not automatically joint-side calibration. Stored effort is
checked against raw current and conversion. Unknown gains are not inferred
simultaneously with unknown masses because absolute scale would be ambiguous.

```python
from urdf2dt.identification import IdentificationData, CurrentCalibration

# q, qd, qdd, time, trajectory_ids and calibrated current come from your log.
# Joint order and source_hash must come from the exact validated robot snapshot.
calibration = CurrentCalibration(gain=(2.0,), offset_ampere=(0.03,),
    effort_units=("N*m",), provenance="Example only: replace with measured calibration record")
effort = calibration.convert(current, ("rad",))
data = IdentificationData(source_hash, ("joint1",), ("rad",), trajectory_ids,
    time, q, qd, qdd, effort, "measured", processing_description, acquisition_id,
    calibration, current)
data.save("training-v1.json")
```

The values above illustrate a one-joint interface; they are not a robot calibration.
The CLI fits saved datasets and saves evaluation separately:

```powershell
python -m urdf2dt.identification fit robot.urdf training-v1.json held-out-v1.json --output outputs/identification/measured-1
```

Supply `--massless-link tool0` only for a genuinely massless known marker and
`--gravity gx gy gz` if gravity differs from `(0,0,-9.81)` in the base frame.
The CLI baseline is explicitly the generic prior. The Python `identify(...,
prior=..., effort_scale=...)` and `evaluate(..., baseline=...)` APIs support
experiment-specific physical priors and predeclared weights. Held-out data never
enters `identify`. Overlapping trajectory IDs and identical relabelled states are
rejected; these checks cannot prove that externally supplied logs are independent.

## Trusted parameters and calibrated motors

```powershell
python -m urdf2dt.identification trust robot.urdf --output trusted-v1.json --provenance "Manufacturer document and revision; acceptance rationale"
```

Trusted mode validates supplied physical parameters and records that fitting was
not performed. Reload checks parameters against the retained original source.
`load_parameters(path)` returns a Stage 24 `DynamicModel` and archive metadata;
its inverse/forward dynamics use the fitted representative. The original URDF
and Stage 24 nominal archive remain unchanged.

`identification.motor.identify_motor` conditionally estimates reflected rotor
inertia, viscous and Coulomb friction with nonnegative least squares. It requires
independently known rigid-body inertias, retained calibrated current observations,
constant motor/joint speed ratios and explicit calibration provenance. Rotor
inertia is reflected inertia divided by speed ratio squared, including a rad/m
ratio for linear transmissions. Training must separate acceleration, velocity
and friction-sign columns. `evaluate_motor` compares an independent holdout.
`save_motor_parameters` / `load_motor_parameters` provide versioned records bound
to the known rigid model; retain that model and the datasets alongside the record.
Motor terms are a separate calibrated residual model, not silently added to the
rigid-body archive. Do not treat a nonunique inertial fit as independently known
rigid-body truth to claim separate rotor identification.

## Benchmark design and acceptance

UR5 and SCARA use the Stage 24 nominal inertias as truth, prescribed nonzero
kinetic friction, and MuJoCo inverse dynamics as the independent effort source.
Four-frequency analytic multisines provide exact positions and derivatives.
Joint position bounds are respected. Training seeds are 2511–2513; held-out seeds
2591–2592; each trajectory has 240 samples over 12 seconds. Independent Gaussian
effort noise has standard deviation 0.001 in each joint's SI effort unit.

The baseline scales all moving-link mass/first moments/inertias by 0.7 and sets
friction to zero. This preserves physical validity while introducing known bias.
The acceptance criteria are fixed before evaluation: fitted held-out RMSE must
improve on that baseline for every joint, and identifiable-combination relative
error must be below 1%. Held-out data is not used to choose priors, penalties,
rank thresholds or hyperparameters. Full raw-parameter recovery is not claimed.

Committed evidence is under `outputs/validation_reports/stage25/`. Reports retain
the source commit, exact source/dataset hashes, dependency versions, parameter
names/units, full training rank/basis/uncertainty and per-joint held-out errors.
The analytical pendulum test additionally recovers its five identifiable inertial/
friction combinations; a calibrated synthetic motor test recovers rotor inertia.
Tests also cover invalid physical values, missing inertias, data corruption,
source/units mismatches, insufficient excitation and holdout leakage.

## Remaining work

Stage 26 trajectory generation is next. Hardware calibration and errors from
sensor differentiation, timing, payload, contact, temperature-dependent friction,
transmission losses and compliance remain experimental concerns for Stages 29–30.
No hardware observations have been fabricated or claimed in these results.
