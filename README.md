3-Axis Robotic Arm Control Using MATLAB & Simulink
Project Overview

This project develops and simulates a 3-axis robotic arm control system using MATLAB and Simulink. The objective is to model the rotational dynamics of three robotic joints and design PID controllers that enable each joint to reach a specified angular position accurately and with minimal oscillation.

The robotic arm consists of three rotational joints:

J1 – Base: Rotates the robot around its vertical axis.
J2 – Shoulder: Controls the main arm elevation.
J3 – Elbow: Controls the forearm movement.

Each joint is modeled using fundamental mechanical dynamics involving inertia, damping, applied torque, and gravity where applicable. PID controllers are then used to reduce the difference between the desired and actual joint angles.

The project focuses on understanding the complete control-design process:

Physical assumptions → Mathematical modeling → Simulink implementation → PID control → Simulation → Performance analysis

The model is currently a simulation-based control study and does not yet include physical hardware.

Objectives

The main objectives of this project are:

Develop a mathematical model for a 3-axis robotic arm.
Represent the mechanical dynamics of each joint in Simulink.
Include inertia and damping effects.
Include gravitational torque for the shoulder and elbow joints.
Design PID controllers for independent joint-angle control.
Provide independent angle references for J1, J2, and J3.
Simulate different joint configurations.
Compare uncontrolled/open-loop and PID-controlled responses.
Evaluate settling time, overshoot, rise time, and steady-state error.
Create a foundation for future real-time robotic-arm implementation.
Robotic Arm Configuration

The simulated robot contains exactly three axes.

J1 – Base Joint

J1 represents the rotational movement of the robot base.

The simplified dynamic equation is:

$$ J_1\ddot{\theta}_1 = \tau_1-b_1\dot{\theta}_1 $$

Therefore,

$$ \ddot{\theta}_1 = \frac{\tau_1-b_1\dot{\theta}_1}{J_1} $$
Assumed Parameters
Parameter	Value	Unit
Joint inertia \(J_1\)	0.08	kg·m²
Damping \(b_1\)	0.04	N·m·s/rad
Gravity	Not included	—

The corresponding Simulink gain is:

$$ \frac{1}{J_1}=\frac{1}{0.08}=12.5 $$
J2 – Shoulder Joint

J2 represents the shoulder movement and includes the effect of gravity.

The assumed dynamic equation is:

$$ J_2\ddot{\theta}_2 = \tau_2-b_2\dot{\theta}_2 -m_2gL_{c2}\sin(\theta_2) $$

Therefore,

$$ \ddot{\theta}_2= \frac{ \tau_2-b_2\dot{\theta}_2 -m_2gL_{c2}\sin(\theta_2) }{J_2} $$
Assumed Parameters
Parameter	Value	Unit
Joint mass \(m_2\)	1.0	kg
Gravity \(g\)	9.81	m/s²
Center-of-mass distance \(L_{c2}\)	0.20	m
Joint inertia \(J_2\)	0.05	kg·m²
Damping \(b_2\)	0.05	N·m·s/rad
Gravity coefficient \(m_2gL_{c2}\)	1.962	N·m

The inverse-inertia gain is:

$$ \frac{1}{J_2}=\frac{1}{0.05}=20 $$

Thus the implemented acceleration relationship is:

$$ \ddot{\theta}_2= 20[ \tau_2 -1.962\sin(\theta_2) -0.05\dot{\theta}_2 ] $$
J3 – Elbow Joint

J3 represents the elbow movement and also includes gravitational torque.

The dynamic equation is:

$$ J_3\ddot{\theta}_3 = \tau_3-b_3\dot{\theta}_3 -m_3gL_{c3}\sin(\theta_3) $$

Therefore,

$$ \ddot{\theta}_3= \frac{ \tau_3-b_3\dot{\theta}_3 -m_3gL_{c3}\sin(\theta_3) }{J_3} $$
Assumed Parameters
Parameter	Value	Unit
Joint mass \(m_3\)	0.60	kg
Gravity \(g\)	9.81	m/s²
Center-of-mass distance \(L_{c3}\)	0.15	m
Joint inertia \(J_3\)	0.025	kg·m²
Damping \(b_3\)	0.03	N·m·s/rad
Gravity coefficient \(m_3gL_{c3}\)	0.8829	N·m

The inverse-inertia gain is:

$$ \frac{1}{J_3}=\frac{1}{0.025}=40 $$

Therefore:

$$ \ddot{\theta}_3= 40[ \tau_3 -0.8829\sin(\theta_3) -0.03\dot{\theta}_3 ] $$
Simulink Model Design

Each joint follows the same basic control principle:

Reference angle → Error calculation → PID controller → Joint dynamics → Actual angle → Feedback

The error is calculated as:

$$ e(t)=\theta_{ref}(t)-\theta(t) $$

The PID controller generates the required joint torque:

$$ \tau(t)= K_pe(t) + K_i\int e(t)dt + K_d\frac{de(t)}{dt} $$

The resulting torque is applied to the corresponding joint dynamic model.

PID Controller Values

The current manually tuned PID values are:

Joint	\(K_p\)	\(K_i\)	\(K_d\)
J1	3	1	0.6
J2	11.2	51	2.2
J3	6	2	1

These values are simulation tuning parameters, not universal physical values. They were selected and adjusted based on the simulated response of each joint.

The controller objective is to achieve:

Accurate target-angle tracking
Low steady-state error
Reduced oscillation
Acceptable settling time
Controlled overshoot
Angle Input

The model allows independent angle commands for the three joints.

For example:

J1 = 30°
J2 = 45°
J3 = 60°

Because the mathematical model operates in radians, the conversion is:

$$ \theta_{rad}=\theta_{deg}\frac{\pi}{180} $$

Therefore:

Joint	Command	Simulink value
J1	30°	0.5236 rad
J2	45°	0.7854 rad
J3	60°	1.0472 rad

This allows different three-axis configurations to be tested without changing the underlying dynamics.

Why These Blocks Are Used
Sum Block

The Sum block calculates the control error:

$$ e=\theta_{ref}-\theta $$

The +- configuration ensures that the actual joint angle is subtracted from the reference angle.

PID Controller

The PID controller converts the angle error into the required control torque.

Gain Block

The gain represents the inverse joint inertia:

$$ \frac{1}{J} $$

For example:

J1 → 12.5
J2 → 20
J3 → 40
Integrators

Two integrators convert acceleration into velocity and then position:

$$ \ddot{\theta} \rightarrow \dot{\theta} \rightarrow \theta $$
Sin Block

The Sin block models the angle-dependent gravitational torque:

$$ \sin(\theta) $$

This is used for J2 and J3.

Damping Gain

The damping term is proportional to angular velocity:

$$ b\dot{\theta} $$

It opposes the motion and represents simplified mechanical friction/damping.

Simulation Method

The project was developed progressively rather than immediately building the complete controller.

Stage 1 — Mathematical Modeling

The mechanical assumptions were defined for each joint.

Stage 2 — Open-Loop Simulation

The joint dynamics were simulated without closed-loop PID control to observe the natural behavior of the system.

This stage helps demonstrate how inertia, damping, and gravity influence the arm.

Stage 3 — Closed-Loop PID Control

Feedback was added and PID controllers were introduced.

The controller continuously compares:

$$ \text{Desired Angle} - \text{Actual Angle} $$

and adjusts the torque accordingly.

Stage 4 — PID Tuning

The PID gains were manually adjusted by observing:

Oscillation
Overshoot
Settling behavior
Final position
Steady-state error
Stage 5 — Multi-Angle Testing

Different target angles were supplied independently to J1, J2, and J3 to verify that the three joints can operate at different commanded positions.

Example Test

One test configuration is:

J1 = 30°
J2 = 45°
J3 = 60°

The expected final configuration is therefore:

$$ \theta_1=30^\circ $$ $$ \theta_2=45^\circ $$ $$ \theta_3=60^\circ $$

The simulation output can then be compared against the reference angles to determine the tracking performance of each joint.

Before and After PID Comparison

The project includes simulation images showing the difference between uncontrolled and PID-controlled behavior.

Before PID Control

The open-loop system demonstrates the natural response of the mechanical model. Depending on the joint, the response can exhibit:

Oscillation
Slow convergence
Gravity-induced displacement
Large position error
Lack of direct target tracking
After PID Control

With feedback control, the system actively corrects the position error.

The response is evaluated using:

Rise time
Settling time
Overshoot
Peak response
Final angle
Steady-state error

The attached screenshots provide visual evidence of the improvement obtained through feedback control.

Performance Evaluation

For each simulation, the following MATLAB analysis can be performed:

ref = 0.7854;

info = stepinfo(theta_out.Data, ...
                theta_out.Time, ...
                ref);

final_angle = theta_out.Data(end);

steady_state_error = ref - final_angle;

steady_state_error_deg = rad2deg(steady_state_error);

This provides quantitative information rather than relying only on visual inspection.

Important metrics include:

Rise Time
Settling Time
Overshoot
Peak
Final Angle
Steady-State Error
Project Scope

This project currently focuses on joint-level robotic-arm control simulation.

It does not yet model:

Full industrial robot dynamics
Motor electrical dynamics
Gearbox backlash
Encoder quantization
Joint friction nonlinearities
Collision detection
Inverse kinematics
Trajectory planning
Physical gripper control
Real-time hardware implementation

These can be added as future extensions.

Future Development

The next development stages can include:

3D robotic-arm visualization
Forward kinematics
Inverse kinematics
Smooth trajectory generation
DC motor and encoder modeling
ESP32-based hardware implementation
Real-time angle feedback
Simulation-versus-hardware comparison
Sensor noise and disturbance testing
Advanced control methods such as computed-torque or adaptive control
Tools & Technologies
MATLAB
Simulink
Simulink Control Design for control-system analysis/tuning where available
PID Control
Classical Control Theory
Mathematical Modeling
Numerical Simulation
Repository Contents

You can organize your GitHub repository like this:

3-axis-robotic-arm-control/
│
├── 3Axis_RoboticArm.slx
├── README.md
│
├── results/
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
Disclaimer / Modeling Assumption

The parameters used in this project are assumed representative values for simulation and educational control-design purposes. They are not intended to represent a specific commercial robotic arm.

The mechanical model is intentionally simplified so that the relationship between physical dynamics, mathematical equations, PID control, and simulation behavior can be studied clearly.
