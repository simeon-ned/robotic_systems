import numpy as np
from visualize import visualize_rotation


def dynamics(state, torque, inertia):
    # \\ System of differential equations to solve
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

# transfomration from quaternion to rotation matrix
def quat2rot(q):
    q0, q1, q2, q3 = q
    rotation_matrix = np.array(
        [[1 - 2 * q1 ** 2 - 2 * q2 ** 2, 2 * q0 * q1 - 2 * q2 * q3, 2 * q0 * q2 + 2 * q1 * q3],
         [2*q0*q1+2*q2*q3, 1 - 2 * q0 ** 2 - 2 *q2 ** 2, 2 * q1 * q2 - 2 * q0 * q3],
         [2 * q0 * q2 - 2 * q1 * q3, 2 * q2 * q1 + 2 * q0 * q3, 1 - 2 * q0 ** 2 - 2 * q1 ** 2]]
    )
    return rotation_matrix


tf = 10
freq = 60 
dT = 1/freq
N = int(tf*freq)

# 
rm = np.zeros([N, 3, 3])
quaternion_norm = np.zeros(N)
time = np.zeros(N)
# initial state 
x = np.array([1., 0., 0., 0., # initial quaternion 
              0.0, 10., 0.01]) # initial angular speed
# system parameters
inertia = np.diag([6, 3, 2]) # inertia matrix

for k in range(N):
    time[k] = dT*k

    # apply "delta" disturbance
    torque = np.zeros(3)
    # if tf/3 < time[k] <= tf/3 + 2*dT:
    #     torque = np.array([3, 3, 0])
    # torque = np.array([0.001, 0, 0])
    # simulate dynamics with forward euler 
    dx = dynamics(x, torque, inertia)
    x += dx*dT
    # normalize quaternion 
    quat_norm = np.linalg.norm(x[:4])
    quat = x[:4]/quat_norm
    x[:4] = quat
    
    # store the quaternion norm and rotation matrices
    quaternion_norm[k] = quat_norm
    rm[k, :, :] = quat2rot(quat)  # np.eye(3)


# ////////////////////////////////////////////////
# animate the rotation matrix and two scalar stats

stats = [[r'time $t$ ', time],
         [r'quat norm $\|q\|$ ', quaternion_norm]]

visualize_rotation(rm,
                   stats=stats, 
                   save = True, 
                   dt = dT)
