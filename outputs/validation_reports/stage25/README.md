# Stage 25 identification evidence

Generated from clean source commit `e87126f188fed0e0dd8963529c3ffb4b44c687f7`.
Both benchmarks passed their predeclared gates. These are simulated observations,
not hardware measurements. See [the complete method and reproduction guide](../../../docs/stages/25_parameter_identification.md).

| Benchmark | Observable combinations / raw parameters | Combination relative error | Held-out RMSE per joint |
| --- | --- | --- | --- |
| UR5 | 48 / 72 | 0.0015455% | 0.000998–0.001044 N*m |
| SCARA | 16 / 58 | 0.0150100% | Revolute: 0.000998–0.001031 N*m; prismatic: 0.000994 N |

Noise standard deviation is 0.001 in each joint's effort unit. Training comprises
three whole trajectories (720 states), evaluation two separate trajectories
(480 states). Each has exact analytic q/v/a. MuJoCo generates effort independently.
The deliberately biased baseline has 70% of nominal inertial values and zero
friction. Every joint improves on its baseline; this is not a comparison against
perfect manufacturer parameters. Unobservable individual inertias are not claimed
as recovered. Reports include full SVD bases and approximate standard errors.

Each robot folder retains checksummed, versioned training/held-out data, fitted
parameters with the exact source URDF, and the benchmark report. Reloading both
parameter archives reproduced their reported held-out RMSE to 1e-12 absolute
tolerance using the reconstructed Stage 24 dynamic model.

Local validation: **370 tests passed**, mypy passed for **56 source files**, and
`pip check` passed in the pinned Python 3.12.14 environment. Separate CLI smoke
checks passed for fitting with a generic prior and trusting explicit source
parameters. The original working-folder edits were preserved in the main checkout.
