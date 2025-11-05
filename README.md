# quintic-trajectory
Python class for quintic polynomial trajectory generation and visualization.

# Quintic Polynomial Trajectory Generator

A lightweight Python implementation for generating **smooth, time-constrained quintic polynomial trajectories** — widely used in robotics, motion planning, and control applications.

This repository demonstrates how to compute **position**, **velocity**, and **acceleration** profiles that satisfy specific boundary conditions (start and end position, velocity, and acceleration).

---

## 📘 Concept Overview

### 🔹 What Are Quintic Polynomials?

A **quintic polynomial** is a fifth-order polynomial of the form:

[
q(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5
]

It provides **continuous position, velocity, and acceleration** — ideal for smooth robotic motion.

By enforcing six boundary conditions:

[
\begin{aligned}
q(t_0) &= q_0, &\quad \dot{q}(t_0) &= \dot{q}_0, &\quad \ddot{q}(t_0) &= \ddot{q}_0, \
q(t_f) &= q_f, &\quad \dot{q}(t_f) &= \dot{q}_f, &\quad \ddot{q}(t_f) &= \ddot{q}_f,
\end{aligned}
]

we obtain a **system of six linear equations** that can be written compactly as:

[
T , C = Q
]

---

## ⚙️ The ( T C = Q ) Formulation

| Symbol | Meaning                                                                                    |
| :----- | :----------------------------------------------------------------------------------------- |
| **T**  | (6 \times 6) matrix of time terms (powers of (t_0) and (t_f))                              |
| **C**  | coefficient vector ([c_0, c_1, c_2, c_3, c_4, c_5]^T)                                      |
| **Q**  | vector of boundary conditions ([q_0, \dot{q}_0, \ddot{q}_0, q_f, \dot{q}_f, \ddot{q}_f]^T) |

Solving ( C = T^{-1} Q ) yields the unique coefficients defining the quintic polynomial.

This structure ensures:

* **C² continuity** (position, velocity, acceleration are continuous)
* **minimum jerk motion** — smooth for actuators and mechanical systems
* easy analytical derivatives for control or simulation

---

## 🧩 Example Usage

```python
from quintic_trajectory import QuinticTrajectory
import numpy as np

# Define boundary conditions
traj = QuinticTrajectory(
    t0=0.0, tf=4.0,
    q0=np.pi/3, dq0=0.0, ddq0=0.0,
    qf=7*np.pi/4, dqf=0.0, ddqf=0.0
)

print("Coefficients:", traj.C)

# Plot position, velocity, acceleration
traj.plot()
```

---

## 📈 Output

* **Position (q)** — smooth transition between start and end waypoints
* **Velocity (dq)** — starts and ends at zero
* **Acceleration (ddq)** — smooth bell-shaped profile

This ensures minimal jerk and feasible torque demand for actuators.

---

## 💡 Practical Insight

In robotic systems, **torque is proportional to angular acceleration**.
Hence, trajectory duration ( T = t_f - t_0 ) must be selected carefully —
short durations cause large accelerations and torque spikes, which may exceed actuator limits.

---

## 🧠 Educational Value

This implementation is ideal for:

* understanding **time-scaling and smooth motion planning**
* validating **simulation results** from MuJoCo, Gazebo, or PyBullet
* serving as a base for **multi-DOF trajectory generation**

---

## 📚 References

* https://www.youtube.com/watch?v=sjlPGj0sHAo
* Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control.*
* Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2005). *Robot Modeling and Control.*
* Siciliano, B. et al. (2010). *Robotics: Modelling, Planning and Control.*

---

## 🧑‍💻 Author

Developed by **Lomash Relia** (M.Tech, Autonomous Systems & Machine Intelligence, ABV-IIITM Gwalior)