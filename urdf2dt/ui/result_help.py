"""Plain-language kinematics conventions shared by result panels."""


def joint_key(chain, symbolic=False):
    joints = [j for j in chain.joints if j.joint_type.value != 'fixed']
    return ', '.join(f'{"q" if symbolic else "J"}{i+1} = {j.name} '
                     f'[{"m" if j.joint_type.value == "prismatic" else "rad"}]'
                     for i, j in enumerate(joints))


def fk_help(reference, target, length_unit, angle_unit):
    return (f'Numerical FK: pose of the origin of "{target}" relative to "{reference}" at the current joint values. '
            f'Position is expressed along the reference axes in {length_unit}; RPY angles use {angle_unit}.\n'
            'The 4×4 matrix maps target-frame coordinates into the reference frame: '
            'upper-left 3×3 = orientation (unitless); last column = position; bottom row = [0 0 0 1]. '
            'Quaternion entries are unitless. Joint input order is listed above.\n'
            'Symbolic FK instead expresses T(q) as a function of joint coordinates. '
            'Displayed rounding does not change calculations; JSON retains full-precision SI values.')


def jacobian_help(reference, target, chain):
    return (f'Numerical geometric Jacobian at the current pose. Target point: origin of "{target}"; '
            f'velocity is relative to and expressed along "{reference}" axes. A moving reference is accounted for.\n'
            'Rows 1–3: linear velocity (vx, vy, vz). Rows 4–6: angular velocity (ωx, ωy, ωz), not RPY rates. '
            'J × joint rates = [v; ω]. Rates use rad/s for rotations and m/s for translations; '
            'output uses m/s and rad/s. Matrix-entry units depend on the row and joint type.\n'
            'Column/source coordinate order: ' + joint_key(chain) + '\n'
            'Symbolic J(q) is a formula over joint coordinates; this numerical table evaluates one pose. '
            'Changing rates updates the predicted relative velocity, not the robot pose.')
