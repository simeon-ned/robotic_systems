#### **Problem : Bicycle**

Consider the following system:


<!-- <p align="center">
<img src="https://drive.google.com/uc?id=191DRVGUMVNG9qGNzdw62jPk3N_JrcXgJ" alt="drawing" width="50%" style="margin:auto"/>
</p> -->


with parameters given by:


```python
m1 = 1.0 # [kg] - mass in the first joint
m2 = 2.0 # [kg] - mass on the end of spring
l = 0.5 # [m]
I1 = 1.0
I2 = 0.1
```

Do the following:

* **[1 points]** Propose the coordinates that will represent configuration of the system, derive the unconstrained dynamics, introduce the appropriate constraints and convert them in to generalized form: $A(q,\dot{q},t) \ddot{q} = b(q, \dot{q}, t)$.

* **[1 points]** Set the external forces $\mathbf{Q}$ to represent the actuation of the system as it shown on the Fig. (b), simulate the behaviour of the system using Runge-Kutta, starting from feasible configuration similar to one on the above figure while letting $F = 4$ and $\tau = 0$, plot the trajectories, and time evoluation of constraints. Animate the resulting motion

* **[1 points]** Change the external forces to represent the given on figure, with $\tau =  5(1.2\sin(t) - \alpha) - \dot{\alpha}$ and $F = 4$, repeat the above steps, 