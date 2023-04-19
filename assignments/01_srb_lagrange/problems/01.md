#### **Problem №1: Lie Integration**


Consider the kinematics of a rotating body parametrized with quaternions, as described by the following ODE:

$$
\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \hat{\boldsymbol{\omega}}
$$

where $\mathbf{q} \in \mathbb{S}^3$ is the unit quaternion and $\hat{\boldsymbol{\omega}} = [0, \boldsymbol{\omega}]$ is the "angular velocity quaternion" with $\boldsymbol{\omega} \in \mathbb{R}^3$ being the angular velocity.

---
Do the following:

* **[10 points]** When selecting a random angular velocity $\boldsymbol{\omega}$, use either the forward or backward Euler method to simulate the motion of the system. Afterwards, check the norm of the quaternion, plot trajectories, and animate the motion, then discuss the results.
* **[10 points]** Instead of Euler integration, apply the following technique: $\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \text{exp}(\frac{1}{2}\hat{\boldsymbol{\omega}}_kdt)$, where $\text{exp}(\mathbf{q})$ is the **quaternion exponential**. Check the norm of the quaternion, plot the trajectories, animate the motion, and compare the results with Euler integration.

---