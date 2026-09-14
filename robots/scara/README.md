# Original RRPR SCARA fixture

`scara_rrpr.urdf` is an original idealized research robot authored for URDF2DT.
It is not a manufacturer model or a measured physical robot. All visual geometry
is embedded URDF primitives; there are no external mesh or ROS package dependencies.

The serial chain has four movable joints: bounded rotary shoulder and elbow,
a downward prismatic slide (0–0.18 m), and a continuous rotary tool joint.
This exercises a different joint count and pattern from the six-revolute-joint UR5.
Parallel and antiparallel axes, coincident axes, fixed mounting/tool transforms,
and metre/radian controls are all exercised.

Arm lengths are 0.35 m and 0.25 m. The world mounting translation is
(0.1, -0.05, 0.2) m with yaw 0.3 rad; the fixed tool offset is -0.08 m in z.
For movable coordinates `(s, e, h, w)`:

```text
x = 0.1  + 0.35 cos(0.3+s) + 0.25 cos(0.3+s+e)
y = -0.05 + 0.35 sin(0.3+s) + 0.25 sin(0.3+s+e)
z = 0.12 - h
yaw = 0.3 + s + e + w
```

The separate research oracle encodes these equations directly without calling the
production URDF parser, DH solver, or transform helpers. Its constants describe
this fixture only and never provide a production fallback.

Visuals are schematic, not mechanical CAD. Inertia, dynamics, self-collision,
actuator feasibility, and manufacturing accuracy are not represented or validated.
