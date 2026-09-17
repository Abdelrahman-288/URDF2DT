# Stage 27 — Robot Controller Layer

Implemented controllers: joint-space PD, PD with measured-position gravity
compensation, and computed torque. Each consumes a Stage 26 `Trajectory`, a
source-matched Stage 24/25 `DynamicModel`, measured position/velocity, and an
explicit fixed-period configuration. Outputs retain desired states, errors,
unclipped effort, clipped effort, and a per-joint saturation flag.

## Equations and units

With `e = q_desired - q` and `edot = v_desired - v`:

- PD: `tau = Kp*e + Kd*edot`.
- PD+gravity: `tau = Kp*e + Kd*edot + g(q)`.
- Computed torque: `a_command = a_desired + Kp*e + Kd*edot`, followed by
  `tau = M(q)*a_command + c(q,v) + g(q) + friction(v)`.

All feedback and dynamics use measured q/v. Computed torque uses the supplied
model, including its recorded friction law. Each effort component is finally
clipped to symmetric finite positive limits. If the trajectory also contains
effort limits, the tighter controller/reference limit applies. Clipping can
invalidate ideal computed-torque cancellation; saturation is always reported.

PD gains have joint-effort/position and joint-effort/velocity units. For a rotary
joint these are N*m/rad and N*m*s/rad; for a linear joint, N/m and N*s/m.
Computed-torque gains instead have units s^-2 and s^-1. They are not numerically
interchangeable with PD gains. Every configuration records mode, diagonal gains,
effort limits and sample period. No integral state or anti-windup is needed for
these three controllers. PI/PID are optional future extensions, not claimed here.

## Timing and failure semantics

`step(time, q, velocity)` accepts consecutive timestamps at integer multiples of
the configured period, beginning at zero. Tolerance is `max(1e-12, period*1e-6)`
seconds. The output is intended for zero-order hold until `hold_until`, which is
capped at reference completion. A command exactly at the endpoint has no remaining
hold interval. No trajectory extrapolation or implicit terminal hold occurs.

Duplicate, skipped, out-of-order or excessive-jitter times, malformed/nonfinite
measurements, nonfinite efforts, and measured positions outside bounded joint
limits are rejected without consuming a tick. Callers must handle exceptions
explicitly; the controller does not silently issue a zero or stale command.
`reset()` intentionally restarts the reference clock at zero. Continuous joints
use unwrapped measured/reference coordinates, preserving multi-turn references.
Measurements are assumed synchronous with the supplied tick; sensor latency and
hardware scheduling are outside this software contract.

## Python usage

```python
from urdf2dt.control import Controller, ControllerConfig
from urdf2dt.dynamics.io import load_dynamic_model
from urdf2dt.trajectory.io import load_trajectory

model = load_dynamic_model("robots/dynamics/scara_dynamics.urdf")
reference = load_trajectory("outputs/validation_reports/stage26/scara/joint_quintic.json")
settings = ControllerConfig("computed_torque", (25.,)*4, (10.,)*4,
                            (30., 30., 100., 10.), period=.01)
controller = Controller(model, reference, settings)
q, velocity, _ = reference.evaluate(0.)
command = controller.step(0., q, velocity)
print(command.effort, command.saturated)
```

Stage 25 `load_parameters()` models also work. Chain hashes, joints, base and tip
must match. `control.io.save_controller()` and `load_controller()` save/load a
versioned checksummed configuration, bound to the exact dynamic parameter/config
digest and reference polynomial digest. Reload starts at tick zero; it does not
resume controller state. Preserve the model and trajectory files with the config.

## Reproduction and sensitivity experiments

Use Python 3.12 with the existing headless environment. No new dependency is
introduced; `requirements/windows-py312-identification.lock` remains the tested
environment. From this stage's checkout:

```powershell
python -m pip install -e ".[dev,research,identification,dynamics-reference]" ipywidgets
python -m pytest -q
python -m mypy
python -m urdf2dt.control scara --output outputs/controllers/scara-1
python -m urdf2dt.control ur5 --output outputs/controllers/ur5-1
```

Use new output directories. Each contains `plant.json`, `reference.json`, nine
controller configuration files, and `controllers.json` with all commands and
the Stage 28 protocol. The plant is fixed at nominal source parameters. Controller
models scale masses and inertia tensors by 0.8, 1.0 and 1.2, preserving COMs,
kinematics and physical consistency. Every model/control mode uses the same
two-second reference and 10ms sampling, prescribed position/velocity offsets,
and explicit demo limits. Gains are illustrative and are not hardware-tuned.

The Stage 27 sweep measures instantaneous acceleration error and saturation at
prescribed states. It is not a closed-loop trajectory simulation, tracking result
or stability claim. Stage 28 will evolve the plant state under the returned
effort and report position/velocity tracking RMSE/max, effort/saturation and
integration timestep convergence, with scenario-specific acceptance criteria
set before simulation. This keeps model-error comparisons reproducible without
using the controller's perturbed model as its own plant.

## Verification and boundaries

Analytical pendulum and prismatic tests cover feedback direction, measured-state
gravity compensation, dynamic cancellation including friction, saturation,
timestamp handling, deterministic reset, rejected inputs, reference acceleration,
terminal behavior, model mismatch and persistent configuration binding. Independent
MuJoCo acceleration checks cover UR5 and SCARA computed-torque outputs. Integration
tests load Stage 25 parameters with Stage 26 references and reproduce both studies.

The module generates bounded software commands; it has no actuator communication,
collision checking or hardware scheduling. The existing Windows executable does
not yet expose the Stage 24–27 Python workflows. Stage 28 is the next stage.
