
#### **Problem №2: Quadrotor Attitude**

Consider the model of quadrotor:
<p align="center">
<img src="https://drive.google.com/uc?id=1S2ZENdiebtmfD4RGuv3mUAVXWoZpAEDL" alt="drawing" width="50%" style="margin:auto"/>
</p>

with following parameters


```python
m = 0.1 # [kg] - mass of the motors 
M = 0.5 # [kg] mass of the drone body
h = 0.08 # [m] high of the body
R = 0.05 # [m] radius of the body
l = 0.3 # [m] length of arm 
k_t = 0.05 # thrust coefficient 
k_m = 0.001 # torque coefficient 
```

Follow the steps below:

* **[10 points]** Derive the dynamics of the drone using the propeller speeds $\Omega_i > 0$ as input (recall the lecture on a single rigid body).

* **[10 points]** Simulate the attitude using normalized quaternions and a 4th-order Runge-Kutta algorithm given random propeller speeds, simulate and animate the motion (recall [practice 3](https://docs.google.com/document/d/134ubHnAKN8aM4MRp0NxQsbsdTSLJSWoZv3mZqFrSRQ/edit)).

* **[20 points]** Deduce the inputs that support rotation around the $x$, $y$, and $z$ axes of the body frame (subsequently), simulate and animate the motion.
---
