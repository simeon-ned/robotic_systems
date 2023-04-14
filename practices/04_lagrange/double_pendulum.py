import numpy as np
from visualize import visualize_double_pendulum

# Define kinematics parameters
lengths = 0.8, 1.2  # length of pendulums

# Add your dynamics parameters here
# masses = [] ....

# the time parameters
t_stop = 3  # how many seconds to simulate
dt = 1/30
t = np.arange(0, t_stop, dt)
# print(t)
# define angles
#
thetas = np.pi+np.sin(2*t), np.cos(2*t)

# calculate cartesian points of joints via forward kinematics
points_1 = lengths[0]*np.sin(thetas[0]), lengths[0]*np.cos(thetas[0])
points_2 = lengths[1]*np.sin(thetas[0]+thetas[1]) + \
    points_1[0], lengths[1]*np.cos(thetas[0]+thetas[1]) + points_1[1]
joint_points = [points_1, points_2]

# Stats to print
stats = [[r'time $t$ ', t],
         [r'norm $\|\mathbf{r}_e\|$ ', np.linalg.norm(np.array(points_2), axis=0)]]  # put energy here

# animate motion of double pendulum
visualize_double_pendulum(joint_points,
                          stats=stats,
                          save=False,  # save as html animation
                          axes=False,
                          show=True,
                          trace_len=0.2,
                          dt=dt)
