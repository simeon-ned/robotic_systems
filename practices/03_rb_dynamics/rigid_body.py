import numpy as np
from visualize import visualize_rotation

# ////////////////////////
# generate rotation matrix

# \\ System of differential equations to solve


def dynamics(state, torque, inertia):
    q, omega = state[:4], state[4:]
    Ic = inertia

    # q0, q1, q2, q3 = q
    omega_x, omega_y, omega_z = omega

    Omega = np.array([[0, -omega_x, -omega_y, -omega_z],
                      [omega_x, 0, omega_z, -omega_y],
                      [omega_y, -omega_z, 0, omega_x],
                      [omega_z, omega_y, -omega_x, 0]])

    dq = Omega @ q / 2
    domega = np.linalg.inv(Ic) @ (torque - np.cross(omega, Ic @ omega))
    return np.hstack((dq, domega))


def quat2rot(q):
    q0, q1, q2, q3 = q
    rotation_matrix = np.array(
        [[1 - 2 * q1 ** 2 - 2 * q2 ** 2, 2 * q0 * q1 - 2 * q2 * q3, 2 * q0 * q2 + 2 * q1 * q3],
         [2*q0*q1+2*q2*q3, 1 - 2 * q0 ** 2 - 2 *q2 ** 2, 2 * q1 * q2 - 2 * q0 * q3],
         [2 * q0 * q2 - 2 * q1 * q3, 2 * q2 * q1 + 2 * q0 * q3, 1 - 2 * q0 ** 2 - 2 * q1 ** 2]]
    )
    return rotation_matrix


N = 2000
tf = 20
dT = tf/N
t = np.linspace(0, tf, N)
rm = np.zeros([N, 3, 3])
quaternion_norm = np.zeros(N)

x = np.array([1., 0., 0., 0., 0., 0., 4])
inertia = np.diag([3, 2, 1])

for k in range(N):
    time = dT*k

    # apply "delta" disturbance
    torque = np.zeros(3)
    if tf/3 < time <= tf/3 + 0.1:
        torque = np.array([0, 3, 0])
    # print()

    dx = dynamics(x, torque, inertia)

    x += dx*dT
    quat_norm = np.linalg.norm(x[:4])
    quat = x[:4]/quat_norm
    x[:4] = quat
    quaternion_norm[k] = quat_norm
    rm[k, :, :] = quat2rot(quat)  # np.eye(3)

# ////////////////////////////////////////////////
# animate the rotation matrix and two scalar stats

stats = [['time', t],
         [r'$|q|$', quaternion_norm]]

visualize_rotation(rm,
                   stats=stats)
