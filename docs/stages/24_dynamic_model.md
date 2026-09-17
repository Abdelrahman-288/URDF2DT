# Stage 24 — Dynamic Model Generation

The headless dynamics module extends the existing validated serial kinematic
chain with independent link inertias, inverse dynamics and forward dynamics.
The Stage 23 Windows EXE remains the tested kinematic candidate; this stage is
available through Python, the application API and the CLI. The final integrated
desktop distribution is part of Stage 31. Stage 23 clean-PC acceptance is deferred
at the project owner's request and is not retroactively marked complete.

## Run

From the Stage 24 checkout, with Python 3.12:

The measured Windows environment is pinned in
`requirements/windows-py312-dynamics.lock`. For exact reproduction, install that
lock into a fresh Python 3.12.14 environment, then `pip install --no-deps -e .`.

```powershell
python -m pip install -e ".[dev,research,dynamics-reference]"
python -m urdf2dt.dynamics robots/dynamics/ur5_dynamics.urdf --output outputs/dynamics/ur5-run-1
python -m urdf2dt.dynamics robots/dynamics/scara_dynamics.urdf --output outputs/dynamics/scara-run-1
```

Each output directory must be new. A passing run writes `dynamic_model.json` and
`validation.json`. A failed numerical comparison writes its report and exits with
status 1 without exporting a validated model. Missing/invalid dynamics data gives
a clear error; kinematic editing remains usable. The original kinematic-only UR5
and SCARA fixtures intentionally continue to lack dynamics data.

```python
from urdf2dt.dynamics.io import load_dynamic_model, load_model_archive

model = load_dynamic_model("robots/dynamics/ur5_dynamics.urdf")
q, v, a = [0.0] * 6, [0.2] * 6, [0.1] * 6
effort = model.inverse_dynamics(q, v, a)
recovered_acceleration = model.forward_dynamics(q, v, effort)
mass_matrix = model.mass_matrix(q)
gravity_effort = model.gravity_effort(q)
restored = load_model_archive("outputs/dynamics/ur5-run-1/dynamic_model.json")
```

An existing `Application` also provides `dynamic_model()` after all DH frames
are accepted and sampled FK passes. It consumes the original URDF link frames,
so legal DH-frame reassignment cannot change the physical inertias.

## Contracts and model

`extract_inertials(URDFInput)` runs independently of the DH solver. Each named
link has a `complete`, `missing`, `incomplete` or `invalid` record. Records retain
mass, the full inertial-origin transform, and all six symmetric tensor values.
No mesh loading occurs. Both the extractor and dynamics consume owned source
bytes with a SHA-256 identity; mismatched link sets or digests are rejected.

The model uses SI units, column-vector transforms, URDF joint signs and ordering.
URDF has no unit tags, so SI is a declared input contract rather than something
automatically inferred from geometry. Gravity defaults to (0, 0, -9.81) m/s² in
the selected base frame. Joint output is N·m for revolute/continuous joints and N
for prismatic joints. Fixed joints retain their transforms and moving descendants'
masses. Inertias for the stationary base/prefix do not affect joint dynamics.
Every moving link, including a fixed child of a moving link, needs complete data.
An explicitly massless marker requires mass=0 and tensor=0; missing data is never
silently converted to a massless link.

For positive mass, the COM tensor must be symmetric, positive definite, and
satisfy principal-inertia triangle inequalities. This excludes degenerate ideal
point/line bodies; analytical fixtures use positive, physically realizable tensors.
Every evaluated mass matrix must admit Cholesky factorization.

The equation is `M(q) a + c(q,v) + g(q) + F(v) = effort`. CasADi differentiates
the link kinetic/potential energies exactly. `c` is the Coriolis/centrifugal vector;
no particular nonunique `C` matrix is exported. Inertia tensors rotate into the
base frame before kinetic energy accumulation. `InertialProperties.in_frame()`
also exposes COM tensors and the parallel-axis shift about a requested origin.

URDF joint damping/friction supplies the default law `b*v + fc*sign(v)`. Missing
coefficients mean zero; negative/nonfinite coefficients are rejected. Explicit
`DynamicsConfig` replaces that policy. `sign(0)=0`: this is kinetic friction,
without a stiction solver. Position/velocity/effort limits are not enforced by
these evaluators. No external wrenches, contacts, payload attachment API, rotor
inertia, transmissions, compliance or floating bases are modeled.

## Independent validation and numerical acceptance

The validation extra pins MuJoCo 3.13.0 (Python 3.11+). MuJoCo is used as an
independent instantaneous reference only; Stage 28's simulator choice remains
open. It reparses the URDF joints and inertias; it does not consume our computed
mass matrix, derivatives or DH table. Geometry, extensions and plugins are
removed from the reference input so no assets or external code are loaded.

The importer is checked against every link pose, mass, COM and tensor. MuJoCo's
[URDF inertial-rotation issue #3559](https://github.com/google-deepmind/mujoco/issues/3559)
was reproduced locally. The adapter independently uses SciPy rotations to express
raw XML tensors in link axes before import and sets only the reference inertial
RPY to zero. The application input remains unchanged. Imported tensor comparison
allows 1e-10 kg m² absolute error for the engine's finite diagonalization precision;
mass and COM checks use 1e-12 absolute/relative tolerance.

Each report saves all sampled positions, velocities, accelerations, efforts,
reference efforts, seed, thresholds, versions, timing and source identity. The
default is 64 samples with seed 2401, including zero and random in-limit positions
(continuous joints use ±pi); velocities and accelerations use ±1 in their joint
SI units. The zero configuration can be outside a robot's position limits.

Default maximum absolute errors are 1e-8 for per-joint effort/acceleration,
1e-9 for mass-matrix entries and pose-matrix entries. Mixed-joint errors are
reported per joint with units; matrix maxima/eigenvalues are numerical checks,
not combined physical norms. Reports include mass symmetry, minimum eigenvalue,
forward/inverse consistency, independent gravity equilibrium and per-joint RMSE.
Friction is checked against a separate scalar analytical law, outside MuJoCo's
stiction/constraint model. Model evaluation timing excludes compilation/reference
work and is not a hard-real-time guarantee.

Analytical tests cover pendulum gravity, coupled two-link mass/Coriolis/gravity,
prismatic motion with a massive fixed tool, dissipation, energy consistency,
rotated/off-diagonal inertias, parallel-axis shifts, missing-data isolation,
legal DH-frame changes, archive integrity and reload.

These are explicit engineering thresholds, not advisor-approved physical accuracy
criteria. Nominal-model numerical agreement is not evidence of hardware accuracy.
Stage 25 parameter identification and later physical experiments remain required.

## Persistence and verification

The dynamics archive is separate from the existing DH session schema. Schema 1.0
stores exact source bytes, their digest, extracted link parameters, joint ordering,
units, gravity/friction configuration, assumptions and a canonical payload hash.
Loading reparses the source, validates FK and physical parameters, compares saved
inertias/order with the source, then rebuilds expressions. The hash detects changes;
it is not a digital signature. Existing files are never overwritten by export.

Run `python -m pytest -q` and `python -m mypy`. Python 3.12 CI explicitly installs
and imports MuJoCo before tests, so independent checks cannot silently disappear.
Python 3.10 continues to test core dynamics and skips only the optional engine file.
See `outputs/validation_reports/stage24/` for the committed numerical evidence.

Sources: [CasADi differentiation](https://web.casadi.org/docs/),
[MuJoCo forward/inverse APIs](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html),
[nominal fixture provenance](../../robots/dynamics/README.md).
