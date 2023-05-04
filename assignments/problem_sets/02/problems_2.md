## **Problem Set 2: Constrained Dynamics and Variational Integrators**

#### **Problem №1: Closed Loop Chain**

Consider the following system:

<p align="center">
<img src="https://simeon-ned.github.io/robotic_systems/assets/images/systems/closed_loop.svg" alt="drawing" width="80%" style="margin:auto"/>
</p>


with parameters given by:


```python
m1 = 0.5 # [kg] - mass in the first joint
m2 = 0.7 # [kg] - mass in the first joint
l = 0.3 # [m] - the length of links
L = 0.4 # [m] - length between fixed joints
g = 9.81 # [m/s^2] the gravitational acceleration
```

Do the following:

* **[5 points]** Choose the coordinates $\mathbf{q}$ to represent the system and derive the free dynamics in these coordinates (you may use Euler-Lagrange or Newton's laws). Write the set of holonomic constraints that represent the closed chain in the form $\boldsymbol{\varphi}(\mathbf{q}) = 0$. Find the constraint Jacobian $\mathbf{J} = \frac{\partial\boldsymbol{\varphi}}{\partial\mathbf{q}}$ and its derivative $\dot{\mathbf{J}}$ as a function of $\mathbf{q}$ and $\dot{\mathbf{q}}$.

* **[15 points]** Use either the Lagrange or Udwadia-Kalaba approach with a Runge-Kutta 4th-order integrator to simulate the motion of the system, given a sample rate of $dt = 0.01$ [s] and a feasible initial point $\mathbf{q}(0)$ in the rest. Calculate and plot the constraint penalization along the simulated trajectory $\|\boldsymbol{\varphi}(\mathbf{q}(t))\|$. Introduce and tune the Baumgarte stabilization technique to reduce constraint penalization. Animate the motion.

* **[15 points]** Introduce the external torques $u_{1,2} = 3\sin(t)$ into the fixed joints as depicted in Figure (b), and repeat the steps above.

---

#### **Problem №2: Bicycle Dynamics**

Consider the following system:

<p align="center">
<img src="https://simeon-ned.github.io/robotic_systems/assets/images/systems/bicycle.svg" alt="drawing" width="80%" style="margin:auto"/>
</p>

with parameters given by:


```python
m1 = 1.0 # [kg] - mass distributed around front wheel
m2 = 2.0 # [kg] - mass distributed around rear wheel
I2 = 0.1 # [kg*m^2] - inertia of front wheel  
I1 = 1.0 # [kg*m^2] - combined inertia of frame and rear wheel
l = 0.5 # [m] - length between wheels
```


Do the following:

* **[10 points]** Propose the coordinates that will represent the configuration of the system, derive the unconstrained dynamics, introduce the appropriate holonomic and nonholonomic constraints, and convert them into a generalized form: $\mathbf{A}(\mathbf{q},\dot{\mathbf{q}},t) \ddot{\mathbf{q}} = \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}, t)$.

* **[15 points]** Set the external forces $\mathbf{Q}$ to represent the actuation of the system as shown in Figure (b). Simulate the behavior of the system using Udwadia-Kalaba approach and Runge-Kutta integrator, starting from a feasible configuration similar to that of Figure (b).  Set $u_1 = 4$ and $u_2 = 0$, plot the trajectories and time evolution of the constraints. Animate the resulting motion, and introduce and tune the Bamguart stabilization.
  
* **[5 points]** Change the actuation forces to represent the given figure (c), with $u_1 = 4$ and $u_2 = 5(1.2\sin(t) - \alpha) - \dot{\alpha}$, then repeat the above steps. Discuss the results.
---


#### **Problem №3: Variational Integrators and Quaternions**

Consider the rotation of single rigid body with associated kinetic energy given by:

$$
    \mathcal{K} = \boldsymbol{\omega}^T \mathbf{I} \boldsymbol{\omega}
$$

With the inertia matrix given with respect to the principle axes as $\mathbf{I} = \text{diag}[I_x, I_y, I_z]$, the inertia parameters are given by:


```python
Ix = 0.1 # [kg*m^2] inertia around x axis 
Iy = 0.2 # [kg*m^2] inertia around y axis
Iz = 0.4 # [kg*m^2] inertia around z axis
```

Do the following:

* **[10 points]** Choose unit quaternions $\mathbf{q} \in \mathbb{S}^3$ to represent the configuration of the system, and define the discrete Lagrangian in these coordinates.
  * *Note*: You may define the angular speed through quaternion derivative as follows: $\hat{\boldsymbol{\omega}} = 2\mathbf{q} \otimes \dot{\mathbf{q}}$, where $\hat{\boldsymbol{\omega}} = [0, \boldsymbol{\omega}]$ and $\mathbf{q}$ is the quaternion conjugate.
* **[15 points]** Derive the constrained discrete Euler-Lagrange equations in momentum form and simulate the system with a sampling rate of $dt = 0.01$ [s], starting from an initial quaternion of $\mathbf{q}(0) = [1,0,0,0]$ and initial velocities of $\omega 1(0)= [6, 0.1, 0.1]$. Repeat the same for $\omega 2(0) = [0.1, 6, 0.1]$ and $\omega 2(0) = [0.1, 0.1, 6]$, animate the trajectories as a rotating frame, and discuss the results.
* **[10 points]** Plot the energy $\mathcal{K}(t)$, the norm of momentum $\|\mathbf{p}(t)\|$, and the norm of the quaternion $\|\mathbf{q}(t)\|$. Compare the results with Euler integration (with and without quaternion normalization).
---
