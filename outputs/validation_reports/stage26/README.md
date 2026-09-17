# Stage 26 evidence

Generated from the Stage 26 branch with the source UR5 and SCARA dynamics
fixtures. Each folder contains checksummed cubic/quintic joint references, a
Cartesian reference, and `report.json` with source hash, limits, path residuals,
Jacobian singularity diagnostics, dynamic effort checks, and dependency versions.

| Robot | Joint quintic | Joint cubic | Cartesian | Cartesian task |
|---|---:|---:|---:|---|
| SCARA | 1.2971 s | 0.5477 s | 0.5373 s | position |
| UR5 | 0.7105 s | 0.3000 s | 0.3013 s | full pose |

The examples use explicit demonstration limits and nominal Stage 24 parameters;
they are controller references only, not manufacturer limits or hardware commands.
Dynamic effort checks are sampled and do not provide a continuous effort or safety
certificate. Stage 27 adds controllers and Stage 28 adds simulation.
