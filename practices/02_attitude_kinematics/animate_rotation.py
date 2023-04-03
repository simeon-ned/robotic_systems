import numpy as np
from visualize import visualize_rotation

# ////////////////////////
# generate rotation matrix

def z_rotation(theta):
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    return R

N = 1000
tf = 5
t = np.linspace(0, tf, N)
rm = np.zeros([N, 3, 3])
theta = np.zeros(N)

for k in range(N):
    theta[k] = 3*t[k]
    rm[k, :, :] = z_rotation(theta[k])


# ////////////////////////////////////////////////
# animate the rotation matrix and two scalar stats

stats = [['time', t], 
         ['theta', theta]]

visualize_rotation(rm, 
                   stats=stats)

