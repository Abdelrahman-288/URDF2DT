# Stage 27 controller evidence

Generated from clean implementation commit
`ea624648f5bb0a71541623829f2a2f03d838ae15`.

UR5 and SCARA each have nine cases: PD, PD+gravity and computed torque, with
controller inertias scaled by 0.8, 1.0 and 1.2. The plant stays nominal. Each case
records 201 commands on a two-second reference at 10ms intervals, with explicit
prescribed position/velocity offsets. The endpoint command has zero hold time.

Each folder contains the original nominal plant archive, Stage 26 reference,
nine controller configurations and the complete `controllers.json` study record.
Every archived configuration was reloaded against its corresponding model and
reference; its first command reproduced the stored command exactly. Reports
include model/reference digests, gains and units, effort limits, saturation flags,
instantaneous acceleration-error metrics and a Stage 28 simulation protocol.

Validation: **413 tests passed**, mypy passed for **68 source files**, and
`pip check` passed in a fresh Python 3.12.14 environment using the Stage 25 lock.
The 25 new controller cases include analytical mechanics, invalid input/timing,
model/configuration persistence, both end-to-end studies and independent MuJoCo
acceleration checks on both robots.

These are prescribed-state software diagnostics. Closed-loop tracking,
integration timestep convergence and stability evaluation belong to Stage 28;
no physical robot observations or hardware-tuned gains are claimed.
