---
layout: home
title: Modeling and Simulation of Robotics Systems
permalink: /
---

### **What this course is about?**
In this course we will explore **simulation** and **modeling techniques** applicable to robotic systems while linking basic knowledge of mechanics to simulation algorithms:

* Recall what a **model** of robotic system is, why it is needed as well as how to build one.
* Consider different approaches to **integration** of dynamics and check the properties of integrators (Implicit/Explicit Euler, Runge-Kutta)
* We will learn how to apply the **Euler-Lagrange equations** to the systems with constraints holonomic and non-holonomic **constraints** (Lagange Multipliers)
* Consider the **numerical algorithms for constrained dynamics** (Udwadia Qalaba)
* Explore and examine the relatively **novel variation integrators**
* Consider the specific features of simulation through **contact**

By the end of the course you will learn how to build **models** of various **robotic systems**: drones, serial and parallel manipulators, mobile robots, etc. You will also exploit different approaches to **simulation** of these systems.

<!-- #### **Core Topics:** -->



#### **Part 1: Introduction, Simulation and Fundamentals** 

* Introduction, Course Outline, History of Dynamics, Models of Robotic Systems
* State Space, Differential Equations, Integrators, Discrete Dynamics and Stability
* Kinematics, Reference Frames
* Rotation matrices, SO(3), Quaternions 

#### **Part 2: Free Dynamics** 
* Dynamics of Rigid Body, Euler Equations
* Angular Momentum, Middle Axis Theorem, Drones, Underwater Vehicles, Centroidal Dynamics
* Variational Approach, Least Action Principle, Statics and Potential Forces, Euler-Lagrange equations, Conserved Quantities
* Equations of Motion, The Properties of EoM, Robotic Manipulators

#### **Part 3: Constrained Dynamics**
* Constraints, Holonomic and Nonholonomic, Lagrange Multipliers
* Parallel Robots, Mobile Robots, Baghuard Stabilization
* Gauss Principle of Least Constraints, Udwadia–Kalaba formulation, Constrained Equation of Motion
<!-- * **Lecture 12** (Week 6): Other View on the Dynamics -->

#### **Part 4: Advanced Integrators and Simulation**  
* Hamiltonian Dynamics and Momentum
* Discrete Least Action and Lagrangian, Verlet and Variational Integrators
* Variational Integrators with Constraints and Non-Potential Forces
* Introduction to Simulation of Contacts
