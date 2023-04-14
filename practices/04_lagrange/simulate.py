import numpy as np
from visualize import visualize_double_pendulum
t_stop = 10  # how many seconds to simulate
dt = 0.01
t = np.arange(0, t_stop, dt)

# define angles
theta_1 = np.sin(t)
theta_2 = np.cos(t)

L1 = 0.8  # length of pendulum 1 in m
L2 = 1.2  # length of pendulum 2 in m
# define cartesian points of joints via forward kinematics
points_1 = L1*np.sin(theta_1), L1*np.cos(theta_1)
points_2 = L2*np.sin(theta_1+theta_2) + points_1[0], L2*np.cos(theta_1+theta_2) + points_1[1]
joint_points = [points_1, points_2]

# Stats to print 
stats = [[r'time $t$ ', t],
         [r'norm $\|\mathbf{r}_e\|$ ', np.linalg.norm(np.array(points_2), axis = 0)]] # put energy here

# Animate
visualize_double_pendulum(joint_points,
                          stats = stats, 
                          axes=False, 
                          dt=dt/2)
