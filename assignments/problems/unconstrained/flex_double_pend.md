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
