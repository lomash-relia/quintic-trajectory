# 🤖 quintic-trajectory
Python class for quintic polynomial trajectory generation and visualization.

# Quintic Polynomial Trajectory Generator

A lightweight Python implementation for generating **smooth, time-constrained quintic polynomial trajectories**, widely used in robotics, motion planning, and control applications.

This repository demonstrates how to compute **position**, **velocity**, and **acceleration** profiles that satisfy specific boundary conditions (start and end position, velocity, and acceleration).

---

## 📘 Concept Overview

### 🔹 What Are Quintic Polynomials?

A **quintic polynomial** is a fifth-order polynomial of the form:

$$
q(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5
$$

It ensures **continuous position, velocity, and acceleration**, making it ideal for smooth robotic motion.

By enforcing six boundary conditions:

$$
\begin{aligned}
q(t_0) &= q_0, &\quad \dot{q}(t_0) &= \dot{q}_0, &\quad \ddot{q}(t_0) &= \ddot{q}_0, \\
q(t_f) &= q_f, &\quad \dot{q}(t_f) &= \dot{q}_f, &\quad \ddot{q}(t_f) &= \ddot{q}_f,
\end{aligned}
$$

we obtain a system of six linear equations that can be written compactly as:

$$
T \, C = Q
$$

---

## ⚙️ The ( T C = Q ) Formulation

| Symbol | Meaning |
| :------ | :------ |
| **T** | 6 × 6 matrix of time terms (refer to the code)|
| **C** | Coefficient vector [c₀, c₁, c₂, c₃, c₄, c₅]ᵀ |
| **Q** | Boundary condition vector [q₀, q̇₀, q̈₀, q_f, q̇_f, q̈_f]ᵀ |

Solving **C = T⁻¹Q** yields the unique coefficients defining the quintic polynomial.


This structure ensures:

- **C² continuity** (position, velocity, and acceleration are continuous)
- **minimum jerk motion**, smooth for actuators and mechanical systems
- easy analytical derivatives for control or simulation

---

## 🧩 Example Usage

```python
!pip install numpy matplotlib -q

from quintic_trajectory import QuinticTrajectory
import numpy as np

# Define boundary conditions
traj = QuinticTrajectory(
    t0=0.0, tf=4.0,
    q0=np.pi/3, dq0=0.0, ddq0=0.0,
    qf=7*np.pi/4, dqf=0.0, ddqf=0.0
)

print("Coefficients:", traj.C)

# Plot position, velocity, and acceleration
traj.plot()
