# Dynamics fixtures

`ur5_dynamics.urdf` preserves the serial fixture's joint/link order, joint origins,
axes and limits, and restores the matching inertial elements from the pinned local
`robots/ur5/ur5_upstream.urdf`. No numerical inertia values are invented or fitted.
The upstream commit, hash, source URL and BSD notices are documented in
[the UR5 fixture record](../ur5/README.md) and [upstream licence](../ur5/LICENSE.upstream).
The fixed `tool0` marker explicitly has zero mass and zero inertia. Missing world
inertia is irrelevant to fixed-base joint dynamics. These are nominal upstream
parameters, not experimentally verified UR5 parameters.

`scara_dynamics.urdf` preserves the original SCARA geometry and adds original,
synthetic nominal parameters for regression/generalization. Base/upper arm/forearm/
carriage/wrist/tool masses are respectively 5, 2, 1.5, 0.8, 0.3, 0.2 kg. COMs
follow the visual origins. Every tensor, in kg m², has diagonal (0.01, 0.012, 0.008)
and off-diagonal (ixy, ixz, iyz)=(0.001, -0.0005, 0.0004), with inertial RPY
(0.2, -0.1, 0.3) rad. These deliberately exercise rotated full tensors and a
massive fixed tool; they do not estimate an actual SCARA's mass distribution.

`scripts/prepare_dynamics_fixtures.py` prints the reproducible XML as a JSON mapping.
The analytical pendulum, two-link arm and prismatic/fixed-tool fixtures are defined
in `tests/test_dynamics.py`, alongside their independently derived equations.

No friction is supplied for these two fixture files; zero friction is the stated
baseline. Stage 25 is responsible for identification and parameter uncertainty.
