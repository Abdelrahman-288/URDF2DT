"""Small rigid-transform operations shared by parsing and numeric kinematics.

Internal helpers assume finite, dimension-checked domain inputs. Matrices multiply
column vectors; composition A @ B applies B first. All returned data are tuples.
"""

from math import cos, hypot, sin

from urdf2dt.dh.types import Transform, Vector


def dot(a: Vector, b: Vector) -> float:
    return sum(x * y for x, y in zip(a, b))


def add(a: Vector, b: Vector) -> Vector:
    return tuple(x + y for x, y in zip(a, b))


def scale(a: Vector, factor: float) -> Vector:
    return tuple(x * factor for x in a)


def subtract(a: Vector, b: Vector) -> Vector:
    return add(a, scale(b, -1.))


def cross(a: Vector, b: Vector) -> Vector:
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def unit(a: Vector) -> Vector:
    return scale(a, 1. / hypot(*a))


def multiply(a: Transform, b: Transform) -> Transform:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)) for i in range(4))


def position(a: Transform) -> Vector:
    return tuple(a[i][3] for i in range(3))


def column(a: Transform, j: int) -> Vector:
    return tuple(a[i][j] for i in range(3))


def rotate(a: Transform, vector: Vector) -> Vector:
    return tuple(dot(row[:3], vector) for row in a[:3])


def inverse(a: Transform) -> Transform:
    translation = position(a)
    return tuple(tuple(a[j][i] for j in range(3)) + (-dot(column(a, i), translation),)
                 for i in range(3)) + ((0., 0., 0., 1.),)


def frame(origin: Vector, x: Vector, z: Vector) -> Transform:
    y = cross(z, x)
    return tuple((x[i], y[i], z[i], origin[i]) for i in range(3)) + ((0., 0., 0., 1.),)


def rpy_transform(xyz: Vector, rpy: Vector) -> Transform:
    """URDF fixed-axis RPY: rotation Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    sr, sp, sy = (sin(v) for v in rpy)
    cr, cp, cy = (cos(v) for v in rpy)
    return (
        (cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, xyz[0]),
        (sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, xyz[1]),
        (-sp, cp*sr, cp*cr, xyz[2]), (0., 0., 0., 1.),
    )


def axis_motion(axis: Vector, q: float, prismatic: bool) -> Transform:
    if prismatic:
        return ((1., 0., 0., axis[0]*q), (0., 1., 0., axis[1]*q),
                (0., 0., 1., axis[2]*q), (0., 0., 0., 1.))
    x, y, z = axis
    c, s = cos(q), sin(q)
    v = 1 - c
    return ((c+x*x*v, x*y*v-z*s, x*z*v+y*s, 0.),
            (y*x*v+z*s, c+y*y*v, y*z*v-x*s, 0.),
            (z*x*v-y*s, z*y*v+x*s, c+z*z*v, 0.), (0., 0., 0., 1.))


def dh_transform(a: float, alpha: float, d: float, theta: float) -> Transform:
    """Standard DH: Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)."""
    ct, st, ca, sa = cos(theta), sin(theta), cos(alpha), sin(alpha)
    return ((ct, -st*ca, st*sa, a*ct), (st, ct*ca, -ct*sa, a*st),
            (0., sa, ca, d), (0., 0., 0., 1.))
