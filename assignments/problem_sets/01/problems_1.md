## **Problem Set 1: Attitude, Rigid Body, Lagrange**

#### **Problem №1: Lie Integration**

Consider the kinematics of a rotating body parametrized with quaternions, as described by the following ODE:

$$
\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \hat{\boldsymbol{\omega}}
$$

where $\mathbf{q} \in \mathbb{S}^3$ is the unit quaternion and $\hat{\boldsymbol{\omega}} = [0, \boldsymbol{\omega}]$ is the "angular velocity quaternion" with $\boldsymbol{\omega} \in \mathbb{R}^3$ being the angular velocity.

Do the following:

* **[10 points]** When selecting a random angular velocity $\boldsymbol{\omega}$, use either the forward or backward Euler method to simulate the motion of the system. Afterwards, check the norm of the quaternion, plot trajectories, and animate the motion, then discuss the results.
* **[10 points]** Instead of Euler integration, apply the following technique: $\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \text{exp}(\frac{1}{2}\hat{\boldsymbol{\omega}}_kdt)$, where $\text{exp}(\mathbf{q})$ is the **quaternion exponential**. Check the norm of the quaternion, plot the trajectories, animate the motion, and compare the results with Euler integration.

---


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

#### **Problem №3: Springy Double Pendulum**

Consider the following system:


<p align="center">
<img src="https://drive.google.com/uc?id=191DRVGUMVNG9qGNzdw62jPk3N_JrcXgJ" alt="drawing" width="50%" style="margin:auto"/>
</p>


with parameters given by:


```python
m1 = 0.5 # [kg] - mass in the first joint
m2 = 1.0 # [kg] - mass on the end of spring
l1 = 1.0 # [m] - the length of second link
l2 = 0.8 # [m] - the length of spring at the rest
k = 200 # [N/m] the stifness of the spring
g = 9.81 # [m/s^2] the gravitational acceleration
```

Do the following:

* **[15 points]** Derive the dynamics of the system using Euler-Lagrange equations, and write the dynamics in the form: $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{Q}$.

* **[15 points]** Set the external forces to zero: $\mathbf{Q} = \boldsymbol{0}$, and simulate the motion using a 4th-order Runge-Kutta algorithm, such that the system was initially at rest with the cartesian position of the second mass $\mathbf{r} 2 = [1.8, 0]$. Animate the resulting motion, plot the trajectories and the full energy of the system ( recall [practice 4](https://docs.google.com/document/d/17JVy8mS9g2jEDtrdaJnYGX3pBBGFVm5pjCG2oQ3rrS0/edit) )

* **[10 points]** Set the external torque on the first joint as follows: $\mathbf{Q}_1(t) = 10(\sin(t) - \mathbf{q}_1) - 0.2\dot{\mathbf{q}}_1$ and repeat the previous step.
