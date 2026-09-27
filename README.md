# 3-Axis Robotic Arm Control Using MATLAB & Simulink

A simulation-based control-system project that models and controls a **3-axis rotational robotic arm** using **MATLAB and Simulink**. The project focuses on mathematical modeling of joint dynamics, PID controller design, closed-loop position control, and quantitative performance analysis.

---

## 📌 Project Overview

This project develops and simulates a **3-axis robotic arm control system** using MATLAB and Simulink.

The objective is to model the rotational dynamics of three robotic joints and design PID controllers that enable each joint to reach a specified angular position accurately and with minimal oscillation.

The robotic arm consists of three rotational joints:

| Joint  | Name     | Function                                   |
| ------ | -------- | ------------------------------------------ |
| **J1** | Base     | Rotates the robot around its vertical axis |
| **J2** | Shoulder | Controls the main arm elevation            |
| **J3** | Elbow    | Controls the forearm movement              |

Each joint is modeled using simplified mechanical dynamics involving:

* Joint inertia
* Mechanical damping
* Applied torque
* Gravitational torque where applicable

PID controllers are then used to minimize the difference between the desired and actual joint angles.

### Control-Design Process

```text
Physical Assumptions
        ↓
Mathematical Modeling
        ↓
Simulink Implementation
        ↓
PID Controller Design
        ↓
Simulation
        ↓
Performance Analysis
```

> **Current Scope:** This is a simulation-based control study and does not currently include physical robotic-arm hardware.

---

# 🎯 Objectives

The main objectives of this project are:

* Develop a mathematical model for a 3-axis robotic arm.
* Represent the mechanical dynamics of each joint in Simulink.
* Include inertia and damping effects.
* Include gravitational torque for the shoulder and elbow joints.
* Design PID controllers for independent joint-angle control.
* Provide independent angle references for J1, J2, and J3.
* Simulate different joint configurations.
* Compare open-loop and PID-controlled responses.
* Evaluate:

  * Rise time
  * Settling time
  * Overshoot
  * Peak response
  * Final angle
  * Steady-state error
* Establish a foundation for future real-time robotic-arm implementation.

---

# 🤖 Robotic Arm Configuration

The simulated robot contains exactly **three rotational axes**.

## J1 — Base Joint

J1 represents the rotational movement of the robot base around the vertical axis.

### Dynamic Equation

$$
J_1\ddot{\theta}_1 = \tau_1 - b_1\dot{\theta}_1
$$

Therefore:

$$
\ddot{\theta}_1 =
\frac{\tau_1-b_1\dot{\theta}_1}{J_1}
$$

### Parameters

| Parameter             |        Value | Unit      |
| --------------------- | -----------: | --------- |
| Joint inertia ($J_1$) |         0.08 | kg·m²     |
| Damping ($b_1$)       |         0.04 | N·m·s/rad |
| Gravity               | Not included | —         |

The inverse-inertia gain is:

$$
\frac{1}{J_1} =
\frac{1}{0.08}
= 12.5
$$

Therefore, the acceleration relationship implemented in Simulink is:

$$
\ddot{\theta}_1 =
12.5(\tau_1-0.04\dot{\theta}_1)
$$

---

# J2 — Shoulder Joint

J2 represents the shoulder movement and includes gravitational torque.

### Dynamic Equation

$$
J_2\ddot{\theta}_2 =
\tau_2-b_2\dot{\theta}_2
-m_2gL_{c2}\sin(\theta_2)
$$

Therefore:

$$
\ddot{\theta}_2 =
\frac{
\tau_2-b_2\dot{\theta}_2
-m_2gL_{c2}\sin(\theta_2)
}{J_2}
$$

### Parameters

| Parameter                          | Value | Unit      |
| ---------------------------------- | ----: | --------- |
| Joint mass ($m_2$)                 |   1.0 | kg        |
| Gravity ($g$)                      |  9.81 | m/s²      |
| Center-of-mass distance ($L_{c2}$) |  0.20 | m         |
| Joint inertia ($J_2$)              |  0.05 | kg·m²     |
| Damping ($b_2$)                    |  0.05 | N·m·s/rad |
| Gravity coefficient ($m_2gL_{c2}$) | 1.962 | N·m       |

The inverse-inertia gain is:

$$
\frac{1}{J_2}
=
\frac{1}{0.05}
=
20
$$

Therefore:

$$
\ddot{\theta}_2 =
20[
\tau_2
-1.962\sin(\theta_2)
-0.05\dot{\theta}_2
]
$$

---

# J3 — Elbow Joint

J3 represents the elbow movement and also includes gravitational torque.

### Dynamic Equation

$$
J_3\ddot{\theta}_3 =
\tau_3-b_3\dot{\theta}_3
-m_3gL_{c3}\sin(\theta_3)
$$

Therefore:

$$
\ddot{\theta}_3 =
\frac{
\tau_3-b_3\dot{\theta}_3
-m_3gL_{c3}\sin(\theta_3)
}{J_3}
$$

### Parameters

| Parameter                          |  Value | Unit      |
| ---------------------------------- | -----: | --------- |
| Joint mass ($m_3$)                 |   0.60 | kg        |
| Gravity ($g$)                      |   9.81 | m/s²      |
| Center-of-mass distance ($L_{c3}$) |   0.15 | m         |
| Joint inertia ($J_3$)              |  0.025 | kg·m²     |
| Damping ($b_3$)                    |   0.03 | N·m·s/rad |
| Gravity coefficient ($m_3gL_{c3}$) | 0.8829 | N·m       |

The inverse-inertia gain is:

$$
\frac{1}{J_3}
=
\frac{1}{0.025}
=
40
$$

Therefore:

$$
\ddot{\theta}_3 =
40[
\tau_3
-0.8829\sin(\theta_3)
-0.03\dot{\theta}_3
]
$$

---

# 🎛️ Simulink Control Architecture

Each joint follows the same basic closed-loop control principle:

```text
Reference Angle
      │
      ▼
 ┌──────────┐
 │   Sum    │◄────────────── Actual Angle
 │ +        │
 │ −        │
 └────┬─────┘
      │
      ▼
 ┌──────────┐
 │   PID    │
 │Controller│
 └────┬─────┘
      │
      ▼
 Applied Torque
      │
      ▼
┌───────────────┐
│ Joint Dynamics│
└───────┬───────┘
        │
        ▼
   Angular Position
        │
        └──────────────► Feedback
```

The control error is:

$$
e(t)=\theta_{ref}(t)-\theta(t)
$$

The PID controller generates the required joint torque:

$$
\tau(t)=
K_pe(t)
+
K_i\int e(t)\,dt
+
K_d\frac{de(t)}{dt}
$$

---

# 🎚️ PID Controller Parameters

The manually tuned PID values currently used in the simulation are:

| Joint  | $K_p$ | $K_i$ | $K_d$ |
| ------ | ----: | ----: | ----: |
| **J1** |     3 |     1 |   0.6 |
| **J2** |  11.2 |    51 |   2.2 |
| **J3** |     6 |     2 |     1 |

These values are **simulation tuning parameters** and are not universal values for physical robotic systems.

They were selected and adjusted based on the simulated response of each joint.

### Controller Goals

The PID controllers are designed to achieve:

* Accurate target-angle tracking
* Low steady-state error
* Reduced oscillation
* Acceptable settling time
* Controlled overshoot

---

# 🎯 Angle Input

The model allows independent angle commands for all three joints.

For example:

```text
J1 = 30°
J2 = 45°
J3 = 60°
```

Because the mathematical model operates in radians, the conversion is:

$$
\theta_{rad} =
\theta_{deg}\frac{\pi}{180}
$$

### Example Conversion

| Joint | Command | Simulink Value |
| ----- | ------: | -------------: |
| J1    |     30° |     0.5236 rad |
| J2    |     45° |     0.7854 rad |
| J3    |     60° |     1.0472 rad |

This allows different three-axis configurations to be tested without changing the underlying mechanical dynamics.

---

# 🧩 Simulink Blocks Used

## Sum Block

The Sum block calculates the control error:

$$
e=\theta_{ref}-\theta
$$

The `+-` configuration ensures that the actual joint angle is subtracted from the reference angle.

---

## PID Controller

The PID controller converts the angular-position error into the required control torque.

It combines:

* Proportional control
* Integral control
* Derivative control

---

## Gain Block

The gain represents the inverse joint inertia:

$$
\frac{1}{J}
$$

The corresponding gains are:

```text
J1 → 12.5
J2 → 20
J3 → 40
```

---

## Integrators

Two integrators convert angular acceleration into angular velocity and then angular position:

$$
\ddot{\theta}
\rightarrow
\dot{\theta}
\rightarrow
\theta
$$

---

## Sin Block

The `Sin` block models the angle-dependent gravitational torque:

$$
\sin(\theta)
$$

It is used for:

* J2 — Shoulder
* J3 — Elbow

---

## Damping Gain

The damping term is proportional to angular velocity:

$$
b\dot{\theta}
$$

It opposes motion and represents simplified mechanical damping/friction.

---

# 🧪 Simulation Methodology

The project was developed progressively rather than immediately building the complete closed-loop controller.

## Stage 1 — Mathematical Modeling

Mechanical assumptions were defined for each robotic joint, including:

* Inertia
* Damping
* Mass
* Center-of-mass distance
* Gravity

The governing differential equations were then derived.

---

## Stage 2 — Open-Loop Simulation

The joint dynamics were initially simulated without closed-loop PID control.

This stage helps demonstrate the natural behavior of the mechanical system and the effects of:

* Inertia
* Damping
* Gravity
* Applied torque

The open-loop response provides a baseline for comparison with the controlled system.

---

## Stage 3 — Closed-Loop PID Control

Feedback was added to the system.

The controller continuously calculates:

$$
\text{Error}
=
\text{Desired Angle}
-
\text{Actual Angle}
$$

The PID controller then adjusts the applied torque to reduce this error.

---

## Stage 4 — PID Tuning

The PID gains were manually adjusted by observing:

* Oscillation
* Overshoot
* Settling behavior
* Final position
* Steady-state error

---

## Stage 5 — Multi-Angle Testing

Different target angles were supplied independently to J1, J2, and J3.

This verifies that the three joints can operate at different commanded positions.

---

# 🔬 Example Test Configuration

One example test configuration is:

```text
J1 = 30°
J2 = 45°
J3 = 60°
```

The expected final configuration is:

$$
\theta_1=30^\circ
$$

$$
\theta_2=45^\circ
$$

$$
\theta_3=60^\circ
$$

The simulation output can then be compared with the reference angles to determine the tracking performance of each joint.

---

# 📊 Before and After PID Comparison

The project includes simulation outputs showing the difference between uncontrolled and PID-controlled behavior.

## Before PID Control

The open-loop system demonstrates the natural response of the mechanical model.

Depending on the joint, the response can exhibit:

* Oscillation
* Slow convergence
* Gravity-induced displacement
* Large position error
* Lack of direct target tracking

---

## After PID Control

With feedback control, the system continuously corrects the position error.

The controlled response can be evaluated using:

* Rise time
* Settling time
* Overshoot
* Peak response
* Final angle
* Steady-state error

Simulation plots provide visual evidence of the effect of feedback control.

---

# 📈 Performance Evaluation

MATLAB can be used to calculate quantitative performance metrics instead of relying only on visual inspection.

Example:

```matlab
ref = 0.7854;

info = stepinfo(theta_out.Data, ...
                theta_out.Time, ...
                ref);

final_angle = theta_out.Data(end);

steady_state_error = ref - final_angle;

steady_state_error_deg = rad2deg(steady_state_error);
```

### Important Performance Metrics

| Metric                 | Purpose                                                                 |
| ---------------------- | ----------------------------------------------------------------------- |
| **Rise Time**          | Time required for the response to reach the target region               |
| **Settling Time**      | Time required for the response to remain within the specified tolerance |
| **Overshoot**          | Amount by which the response exceeds the target                         |
| **Peak Response**      | Maximum angular response                                                |
| **Final Angle**        | Final achieved joint position                                           |
| **Steady-State Error** | Difference between reference and final position                         |

These metrics can be recorded separately for J1, J2, and J3.

---

# 📁 Suggested Repository Structure

```text
3-axis-robotic-arm-control/
│
├── 3Axis_RoboticArm.slx
├── README.md
│
├── results/
│   │
│   ├── before_pid/
│   │   ├── J1_before_PID.png
│   │   ├── J2_before_PID.png
│   │   └── J3_before_PID.png
│   │
│   ├── after_pid/
│   │   ├── J1_after_PID.png
│   │   ├── J2_after_PID.png
│   │   └── J3_after_PID.png
│   │
│   └── final_robot_output.png
│
└── documentation/
    └── parameter_reference.md
```

---

# 🛠️ Tools & Technologies

| Technology                   | Purpose                                             |
| ---------------------------- | --------------------------------------------------- |
| **MATLAB**                   | Mathematical modeling and numerical analysis        |
| **Simulink**                 | Dynamic system modeling and simulation              |
| **PID Control**              | Closed-loop joint position control                  |
| **Simulink Control Design**  | Control-system analysis and tuning, where available |
| **Classical Control Theory** | Controller design and performance analysis          |
| **Numerical Simulation**     | Evaluation of robotic joint responses               |

---

# 🚧 Current Project Scope

This project currently focuses on **joint-level robotic-arm control simulation**.

The present model does **not** include:

* Full industrial robot dynamics
* Motor electrical dynamics
* Gearbox backlash
* Encoder quantization
* Detailed nonlinear joint friction
* Collision detection
* Inverse kinematics
* Trajectory planning
* Physical gripper control
* Real-time hardware implementation

These features can be introduced in future versions.

---

# 🚀 Future Development

Possible extensions include:

### 1. 3D Robotic-Arm Visualization

Develop a 3D representation of the robotic arm and visualize joint movements in real time.

### 2. Forward Kinematics

Calculate the end-effector position and orientation from the three joint angles.

### 3. Inverse Kinematics

Calculate the required joint angles for a desired end-effector position.

### 4. Smooth Trajectory Generation

Replace simple step commands with smooth trajectories such as:

* Polynomial trajectories
* Trapezoidal velocity profiles
* S-curve profiles

### 5. DC Motor and Encoder Modeling

Extend the mechanical model to include:

* DC motors
* Motor torque constants
* Back EMF
* Encoder feedback
* Motor voltage control

### 6. ESP32-Based Hardware Implementation

Implement the controller on an ESP32 or similar embedded platform and connect it to physical robotic joints.

### 7. Real-Time Angle Feedback

Use encoders to measure actual joint positions and close the loop around real hardware.

### 8. Simulation-vs-Hardware Comparison

Compare:

```text
MATLAB/Simulink Response
          ↓
Physical Robot Response
```

to identify modeling errors and real-world effects.

### 9. Disturbance and Sensor-Noise Testing

Introduce:

* External disturbances
* Sensor noise
* Parameter variations
* Load changes

to evaluate controller robustness.

### 10. Advanced Control

Future research can investigate advanced approaches such as:

* Computed-torque control
* Adaptive control
* State-space control
* Model predictive control
* Robust control

---

# 📌 Modeling Assumptions & Disclaimer

The parameters used in this project are **assumed representative values** intended for simulation and educational control-design purposes.

They are **not intended to represent a specific commercial robotic arm**.

The mechanical model is intentionally simplified so that the relationship between:

```text
Physical Dynamics
       ↓
Mathematical Equations
       ↓
Simulink Model
       ↓
PID Controller
       ↓
Joint Response
```

can be studied clearly.

A real robotic arm would require additional considerations such as actuator dynamics, coupled joint dynamics, gearbox characteristics, nonlinear friction, encoder feedback, structural flexibility, safety constraints, and hardware limitations.

---

# 📚 Project Summary

This project demonstrates the complete fundamental workflow of a robotic joint control system:

```text
         ROBOTIC ARM
              │
      ┌───────┼───────┐
      ↓       ↓       ↓
     J1      J2      J3
    Base   Shoulder  Elbow
      │       │       │
      └───────┼───────┘
              ↓
     Mathematical Model
              ↓
        Simulink Model
              ↓
        PID Controllers
              ↓
       Closed-Loop Control
              ↓
        Simulation Results
              ↓
     Performance Analysis
```

The project provides a foundation for progressing from **simulation-based joint control** toward **kinematics, trajectory planning, real-time sensing, embedded control, and physical robotic-arm implementation**.

---

## 👨‍💻 Project Status

**Status:** Simulation / Control-System Study

**Axes:** 3 rotational joints

**Controller:** Independent PID controllers

**Platform:** MATLAB + Simulink

**Hardware:** Not currently implemented

**Primary Focus:** Joint-angle position control and performance evaluation
