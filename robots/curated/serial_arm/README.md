## Three-joint serial arm

File: `serial_arm/serial_arm.urdf`

3 bounded revolute joints; link lengths 0.35, 0.28 and 0.17 m; 4 body visuals.

![Three-joint serial arm body rendering](../images/serial_arm.png)

**Expected appearance:** A dark cylindrical base with blue, silver and orange solid arm links. Three rotary joints move a single planar chain. This is a teaching arm, not a UR5 or other manufacturer model.

Image pose: joint_1 = 0.4, joint_2 = -0.8, joint_3 = 0.4 rad.

**Exercise:** Open serial_arm/serial_arm.urdf. In Joint motion set joint_1 to 1.5707963267948966 rad (90 degrees) and joint_2/joint_3 to zero. Open FK calculator, use the URDF model, reference base and target tool, and calculate. The target position should be approximately [0.000, 0.800, 0.180] m. Reset the pose: it becomes [0.800, 0.000, 0.180] m. Inspect the orange body and try its opacity slider.

Licence: original URDF2DT teaching geometry under MIT; see ../LICENSE and ../ASSET_LICENCES.md.
