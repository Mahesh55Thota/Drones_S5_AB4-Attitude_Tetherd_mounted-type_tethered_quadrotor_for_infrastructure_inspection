<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="700">
</p>

<h1 align="center">Attitude Control of a Camera-Mounted Tethered Quadrotor</h1>

<p align="center">
  <b>2-DOF Camera Stabilization and Tether-Based Position Sensing</b><br>
  Amrita Vishwa Vidyapeetham
</p>

# Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | G Chaitanya Varma | CB.SC.U4AIE24017 | cb.sc.u4aie24017@cb.students.amrita.edu |
| 2 | M Jaya Krishna Sai |CB.SC.U4AIE24030  | cb.sc.u4aie24030@cb.students.amrita.edu |
| 3 | S Sathwik |CB.SC.U4AIE24051 | cb.sc.u4aie24051@cb.students.amrita.edu|
| 4 | S V Dhiraj | CB.SC.U4AIE24052 | cb.sc.u4aie24052@cb.students.amrita.edu|
| 5 | TMSK Maheswar | CB.SC.U4AIE24058 |cb.sc.u4aie24058@cb.students.amrita.edu |

---

# Abstract

This project presents a control system for a tethered quadrotor with a camera for infrastructure inspection. The work is based on the reference paper by Keigo Watanabe, Nao Moritoki, and Isaku Nagai. The quadrotor is connected to the ground by a tether, which is used for position sensing. The position of the quadrotor is determined using the tether geometry and measured angles. Based on the position, the required drone attitude is controlled.

The project focuses mainly on roll and pitch attitude control of the quadrotor using P and PD controllers. A 2-DOF camera stabilizer is also considered to compensate for the motion of the quadrotor and maintain the camera orientation. PID control is used for the camera's roll and pitch stabilization. The simulation uses the moments of inertia and controller parameters reported in the reference paper.

A **PyBullet-based Gym environment** is developed to simulate the quadrotor control system and generate the corresponding response plots using Python. The simulation represents the major components of the system, including tether-based position estimation, quadrotor attitude dynamics, attitude control, and camera stabilization. It helps analyze how the quadrotor and camera respond to disturbances and evaluate how the controllers maintain the desired position and orientation during infrastructure inspection.
# 1. Introduction

Quadrotors are useful for infrastructure inspection because they can take off vertically, hover, and move easily around structures where it may be difficult to use conventional inspection equipment.

The reference work considers a tethered quadrotor for inspection of structures such as tunnels and bridges. A tether provides a method of determining the quadrotor position without relying on GPS, which can be unavailable in tunnels or beneath bridge structures.

When a camera is directly mounted on the quadrotor, changes in the body attitude can also change the camera field of view. To overcome this problem, the project uses a 2-DOF camera stabilizer that controls camera roll and pitch independently.

The complete simulation therefore focuses on three main parts:

- Tether-based position sensing
- Quadrotor attitude control
- 2-DOF camera attitude stabilization

---

# 2. Objectives

The main objectives of the project are:

### 2.1 Tether-Based Position Sensing

Calculate the quadrotor position from tether length and tether inclination angles.

### 2.2 Quadrotor Attitude Control

Implement attitude control for roll, pitch, and yaw using the control equations described in the reference paper.

### 2.3 P vs PD Controller Comparison

Compare proportional control with proportional-derivative control for roll and pitch attitude stabilization.

### 2.4 Camera Stabilization

Develop a 2-DOF camera stabilizer for independent roll and pitch control.

### 2.5 PID Control

Implement PID control for the camera stabilizer to maintain the camera near the desired attitude.

### 2.6 PyBullet Simulation

Use PyBullet to simulate the quadrotor system and evaluate its control response. Record the system behaviour and generate response plots using Python to analyze how the quadrotor responds to changes in position, orientation, and external disturbances.

### 2.7 Gym Environment

Develop a Gym-compatible environment using PyBullet to organize the quadrotor simulation and test its control behaviour. The environment provides a structured interface for applying control actions, observing the system's response, and evaluating the implemented control components for infrastructure inspection and camera stabilization.

---

# 3. Base Paper

## Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection

The project is based on the study:

**Keigo Watanabe, Nao Moritoki, and Isaku Nagai, "Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection."**

The paper develops and experimentally evaluates a tethered quadrotor for infrastructure inspection. It specifically describes quadrotor attitude control and a 2-DOF camera stabilizer.

### Main Contributions of the Paper

- Tether-based position detection
- Quadrotor attitude control using PD control
- 2-DOF camera stabilizer
- Camera roll and pitch stabilization
- Experimental comparison of P and PD attitude control
- Experimental verification of camera stabilization

### Relation to the Present Project

The present project implements the mathematical control equations and reported physical parameters from the paper in MATLAB and Simulink.

The project is a simulation-based implementation of the control concepts and is not claimed to reproduce every physical detail of the experimental hardware.

---

# 4. Methodology

## 4.1 Simulation Environment

The project is implemented using:

- Python
- PyBullet physics simulator
- Gym-compatible environment
- Python plotting and analysis tools

The PyBullet-based simulation models the quadrotor system, while the Gym-compatible environment provides a structured interface for applying control actions and observing the system's response. Python is used to analyze the simulation results and generate plots to evaluate the quadrotor's behaviour and the implemented control components.

---

## 4.2 Quadrotor Model

The quadrotor has four rotors and is represented using its principal moments of inertia.

The measured moments of inertia used in the project are:

| Parameter | Value | Unit |
|---|---:|---|
| Mass | 1.2 | kg |
| Gravity | 9.81 | m/s² |
| Tether length | 0.25 | m |
| Ix | 0.01910 | kg·m² |
| Iy | 0.01910 | kg·m² |
| Iz | 0.03083 | kg·m² |

The inertia values correspond to the measured values reported in Table I of the reference paper.

---

## 4.3 Tether Position Sensing

The tether inclination angles are represented by:

- `alpha` — inclination in the Ex direction
- `beta` — inclination in the Ey direction
- `l` — tether length

The position equations are:

$$
x = z\tan(\alpha)
$$

$$
y = z\tan(\beta)
$$

The vertical position is calculated using:


$$
z=-\sqrt{
\frac{
l^2(\cos\alpha)^2(\cos\beta)^2
}{
(\cos\alpha)^2+(\cos\beta)^2-(\cos\alpha)^2(\cos\beta)^2
}
}
$$

```text
tether_pos_func.m
```

For the verification case used in `main.m`:

```text
Tether length = 0.25 m
alpha = 15°
beta  = 10°
```

the calculated values are approximately:

```text
z = -0.2381 m
x = -0.0638 m
y = -0.0420 m
```

---

# PD and PID Controllers

## 1. PD Controller — Quadrotor Attitude and Position Control

### What is a PD Controller?

PD stands for **Proportional-Derivative**. It is a feedback controller that uses two terms to calculate a corrective control action:

- **Proportional (P) term:** Responds to the current error between the desired value and the actual value.
- **Derivative (D) term:** Responds to the rate of change or motion, helping to damp oscillations and reduce excessive movement.

In simple words:

- **P:** How far am I from the target?
- **D:** How quickly am I moving, and how can I reduce excessive motion?

The main purpose of PD control in this project is to make the quadrotor follow the desired attitude and position with reduced error and oscillation.

### 1.1 Quadrotor Attitude PD Controller

In the reference paper, the quadrotor attitude controller controls three rotational directions:

- Roll (`φ`): Tilting left or right.
- Pitch (`θ`): Tilting forward or backward.
- Yaw (`ψ`): Rotating around the vertical axis.

The desired angles are represented by `φd`, `θd`, and `ψd`. The current angles are represented by `φ`, `θ`, and `ψ`.

The paper uses the following PD control equations.

**Roll control:**

$$
U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}
$$

**Pitch control:**

$$
U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}
$$

**Yaw control:**

$$
U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}
$$

Here:

- `U2`, `U3`, and `U4` are the roll, pitch, and yaw control inputs.
- `φ`, `θ`, and `ψ` are the current attitude angles.
- `φd`, `θd`, and `ψd` are the desired attitude angles.
- `φ̇`, `θ̇`, and `ψ̇` are the corresponding angular velocities.
- `K1`, `K3`, and `K5` are proportional gains.
- `K2`, `K4`, and `K6` are derivative gains.

The controller calculates the control inputs from the attitude errors and angular velocities. These inputs are passed to the motor-mixing stage, which determines the commands sent to the four rotors.

#### How Does the Attitude PD Controller Work?

Suppose the desired roll angle is `10°`, but the actual roll angle is `4°`. The controller detects that the quadrotor has not yet reached the desired angle.

The proportional term responds to this angle error. The derivative term considers how quickly the quadrotor is rolling and helps reduce excessive rotation.

The controller calculates a control input, and the motor commands are adjusted to produce the required roll torque. The quadrotor rotates toward the desired angle, its new attitude is measured, and the controller recalculates the control input.

This process repeats continuously through feedback.

#### Role of the Attitude Gains

| Gain | Purpose |
|---|---|
| `K1` | Proportional gain for roll |
| `K2` | Derivative gain for roll |
| `K3` | Proportional gain for pitch |
| `K4` | Derivative gain for pitch |
| `K5` | Proportional gain for yaw |
| `K6` | Derivative gain for yaw |

Increasing a proportional gain generally strengthens the correction for a given error. Increasing a derivative gain generally adds damping. However, excessively large gains can cause instability, oscillation, or sensitivity to noisy measurements.

### 1.2 Quadrotor Position PD Controller

The reference paper also uses a PD-style controller to control the quadrotor's horizontal position.

Instead of directly generating motor commands, the position controller calculates the desired pitch and roll angles required to move the quadrotor toward its target.

**X-direction position control:**

$$
\theta_d=-K_7(x-x_d)-K_8\dot{x}
$$

**Y-direction position control:**

$$
\phi_d=-K_9(y-y_d)-K_{10}\dot{y}
$$

Here:

- `x`, `y` are the current horizontal position coordinates.
- `xd`, `yd` are the desired horizontal position coordinates.
- `ẋ`, `ẏ` are the current velocities along the X and Y directions.
- `θd` is the desired pitch angle.
- `φd` is the desired roll angle.
- `K7` and `K9` are proportional position gains.
- `K8` and `K10` are velocity-feedback gains.

The current position can be estimated from the tether geometry described in the paper. The controller compares the current and desired positions and calculates the required attitude commands.

For example, if the quadrotor is far from its desired X position, the proportional term generates a stronger pitch correction. The velocity term considers how quickly the quadrotor is moving and helps reduce excessive motion and overshoot.

The desired pitch and roll are passed to the attitude controller, which then generates the control inputs used by the rotor system.

The complete position and attitude control sequence is:

**Tether measurements → Position estimation → Position PD controller → Desired pitch and roll → Attitude PD controller → Motor mixing → Quadrotor movement → Updated feedback.**

The paper keeps the vertical control input `U1` constant so that the tether remains stretched. Therefore, this position controller should not be described as a complete three-axis position controller.

The paper does not provide a detailed derivation for the position gains `K7` to `K10` in its gain-selection section. These gains must therefore be documented and tuned separately in our simulation.

### 1.3 Gain Selection and Tuning for PD Control

The reference paper uses an inertia-based method to obtain initial proportional gains for the quadrotor attitude controller. It then increases and fine-tunes the gains experimentally. The derivative gains are adjusted manually.

The paper reports the following attitude PD settings in its experiments:

| Attitude axis | Proportional gain | Derivative gain |
|---|---:|---:|
| Roll | `K1 = 0.625` | `K2 = 0.170` |
| Pitch | `K3 = 0.810` | `K4 = 0.340` |

These values provide reference settings for our simulation. Since simulated dynamics and actuator models may differ from the physical quadrotor, the controller must be tested and tuned in the simulation environment.

---

## 2. PID Controller — Camera Stabilization

### What is a PID Controller?

PID stands for **Proportional-Integral-Derivative**. It is a feedback controller that combines three terms to calculate a corrective control action:

- **Proportional (P) term:** Responds to the current error.
- **Integral (I) term:** Accumulates error over time and helps reduce persistent steady-state error.
- **Derivative (D) term:** Responds to how quickly the error changes and provides damping.

In simple words:

- **P:** How large is the error right now?
- **I:** How long has the error persisted, and how much has accumulated?
- **D:** How quickly is the error changing?

The main purpose of PID control in this project is to stabilize the camera's orientation while the quadrotor changes its attitude during infrastructure inspection.

### 2.1 Camera Stabilizer in the Reference Paper

The paper describes a **2-DOF camera stabilizer** that controls camera roll and pitch.

The stabilizer contains:

- A camera mounted on a mechanical gimbal.
- Two RC servo motors for changing the camera attitude.
- An MPU-9150 inertia sensor attached to the camera to detect its attitude.

The stabilizer controls two axes:

- Camera roll (`φC`).
- Camera pitch (`θC`).

The yaw drive of the camera stabilizer was removed to reduce weight and energy consumption. The paper assumes that yaw movement can be handled by the main quadrotor body.

### 2.2 Current and Desired Camera Angles

The current camera angles are represented by:

- `φC`: Current camera roll.
- `θC`: Current camera pitch.

The desired camera angles are represented by:

- `φCd`: Desired camera roll.
- `θCd`: Desired camera pitch.

The desired angles are reference values representing the orientation the camera should maintain during inspection.

The paper states that an inertia sensor is attached to the camera and used to detect its attitude. The sensor specified in the physical implementation is the MPU-9150.

In our simulation, a simulated camera-mounted IMU can provide the equivalent attitude feedback.

### 2.3 Camera Attitude Errors

The paper defines the camera roll and pitch errors as follows.

**Camera roll error:**

$$
e_{\phi_C}=\phi_{Cd}-\phi_C
$$

**Camera pitch error:**

$$
e_{\theta_C}=\theta_{Cd}-\theta_C
$$

For example, suppose the desired camera roll is `0°`, but the current camera roll is `8°`.

The roll error is:

$$
e_{\phi_C}=0^\circ-8^\circ=-8^\circ
$$

This error indicates that the actual camera attitude differs from the desired reference.

### 2.4 Camera PID Control Equations

The paper uses PID control for both camera axes.

**Camera roll PID controller:**

$$
U_{\phi_C}=
K_{11}e_{\phi_C}
+K_{12}\int e_{\phi_C}dt
+K_{13}\dot{e}_{\phi_C}
$$

**Camera pitch PID controller:**

$$
U_{\theta_C}=
K_{14}e_{\theta_C}
+K_{15}\int e_{\theta_C}dt
+K_{16}\dot{e}_{\theta_C}
$$

Here:

- `UφC` is the camera roll control input.
- `UθC` is the camera pitch control input.
- `K11` and `K14` are proportional gains.
- `K12` and `K15` are integral gains.
- `K13` and `K16` are derivative gains.
- `eφC` and `eθC` are the camera roll and pitch errors.
- `ėφC` and `ėθC` are the rates of change of those errors.

The controller outputs are mapped to the two camera stabilizer actuators in the physical system or to the corresponding gimbal-joint actuators in the simulation.

### 2.5 How the Three PID Terms Work

#### Proportional Term (P)

The proportional term responds to the current camera error.

For example, if the camera is tilted far away from its desired orientation, the proportional term requests a stronger correction.

$$
P=K_Pe(t)
$$

The P term helps the camera respond to deviations from its target.

#### Integral Term (I)

The integral term accumulates the error over time.

$$
I=K_I\int e(t)dt
$$

Suppose the camera retains a small error for a long time. The integral term accumulates this error and can provide additional correction to reduce the remaining steady-state error.

In simulation, integral windup protection should be implemented to prevent the accumulated error from becoming excessive when a servo reaches its limits.

#### Derivative Term (D)

The derivative term responds to the rate at which the error changes.

$$
D=K_D\frac{de(t)}{dt}
$$

It provides damping and can help reduce overshoot and oscillations. In an implementation, derivative filtering may be useful because directly differentiating noisy measurements can amplify sensor noise.

### 2.6 How the Camera Stabilizer Works

Suppose the quadrotor rolls while moving near a tunnel wall. Since the camera is mounted on the quadrotor, the camera would tend to roll with the body if the stabilizer did nothing.

The camera-mounted IMU detects the change in camera attitude. The controller compares the measured camera roll and pitch with their desired reference values and calculates the errors.

The two PID controllers calculate `UφC` and `UθC`. These commands drive the two stabilizer actuators in the corrective direction. The camera then moves toward the desired orientation.

The IMU detects the new camera attitude, the errors are recalculated, and the controller updates the actuator commands.

The feedback loop is:

**Quadrotor movement → Camera attitude disturbance → Camera IMU measurement → Error calculation → PID controller → Servo commands → Camera correction → New measurement.**

This process continuously repeats so that the camera orientation remains as close as possible to the desired reference, even when the quadrotor tilts.

### 2.7 Camera PID Gain Settings

The paper reports the following camera PID gains, selected through trial and error in the physical experiment:

| Camera axis | P gain | I gain | D gain |
|---|---:|---:|---:|
| Roll | `K11 = 170.0` | `K12 = 2.0` | `K13 = 0.1` |
| Pitch | `K14 = 170.0` | `K15 = 2.0` | `K16 = 0.1` |

These values are reference settings from the paper. They must be verified and, if necessary, retuned in our simulation if the actuator model, units, scaling, or dynamics differ from the physical setup.

---

## 3. Main Difference Between PD and PID Control

| Feature | PD Controller | PID Controller |
|---|---|---|
| Terms used | Proportional + Derivative | Proportional + Integral + Derivative |
| Current error | Yes | Yes |
| Accumulated error | No | Yes |
| Rate of change | Yes | Yes |
| Main role in this project | Quadrotor attitude and horizontal position control | Camera roll and pitch stabilization |
| Control objective | Move the quadrotor toward the desired position and attitude | Keep the camera close to its desired orientation |
| Output | Desired roll/pitch from position control, or attitude control inputs | Camera roll/pitch control inputs |

---

## 4. How Both Controllers Work Together

The position controller and attitude controller form the quadrotor control chain.

The position controller uses the tether-based position estimate to calculate desired pitch and roll. The attitude PD controller then uses the desired and current attitude to calculate the control inputs for the rotor system.

At the same time, the camera PID controller independently uses camera attitude feedback to correct camera roll and pitch.

The overall architecture is:

**Quadrotor control:**

Tether geometry → Position estimation → Position PD controller → Desired pitch and roll → Attitude PD controller → Motor mixing → Quadrotor movement.

**Camera stabilization:**

Camera-mounted IMU → Current camera attitude → Camera error calculation → Camera PID controller → Gimbal actuators → Camera stabilization.

In our Gazebo or MuJoCo implementation, simulated sensors and actuators can be used to reproduce these control structures.

Tunnel navigation, crack detection, defect tracking, and defect localization are additional project extensions. They should be documented separately from the original paper's controller design.

## 5. Key Takeaway

The **PD controller** helps the quadrotor achieve the desired position and attitude by using error and motion feedback. The **PID controller** stabilizes the camera by using current error, accumulated error, and the rate of error change to command the camera gimbal.

Both controllers operate continuously in feedback loops: they measure the system, calculate errors, generate control commands, apply corrections, and repeat the process.

## 4.4 Generation of Desired Roll and Pitch

The tether provides the current position of the quadrotor as $(x,y,z)$. The desired position is defined by $(x_d,y_d)$.

The reference paper explains that the quadrotor can move in the $E_X$ direction by changing the pitch angle $\theta$, and in the $E_Y$ direction by changing the roll angle $\phi$. Therefore, position control is achieved by generating suitable desired pitch and roll angles for the inner attitude controller. :contentReference[oaicite:1]{index=1}

Let:

- $x$ = current position in the $E_X$ direction
- $y$ = current position in the $E_Y$ direction
- $x_d$ = desired position in the $E_X$ direction
- $y_d$ = desired position in the $E_Y$ direction
- $\dot{x}$ = velocity in the $E_X$ direction
- $\dot{y}$ = velocity in the $E_Y$ direction
- $K_7,K_8,K_9,K_{10}$ = position-control gains

The desired pitch angle is calculated using the position error in the $E_X$ direction:

$$
\theta_d=-K_7(x-x_d)-K_8\dot{x}
$$

The desired roll angle is calculated using the position error in the $E_Y$ direction:

$$
\phi_d=-K_9(y-y_d)-K_{10}\dot{y}
$$

These are the position-control equations (7) and (8) given in the reference paper. :contentReference[oaicite:2]{index=2}

The equations contain two terms:

### Desired Pitch

$$
\theta_d=-K_7(x-x_d)-K_8\dot{x}
$$

The first term,

$$
-K_7(x-x_d)
$$

is the proportional position correction. It depends on how far the quadrotor is from the desired $x$ position.

The second term,

$$
-K_8\dot{x}
$$

provides velocity feedback and helps reduce excessive motion.

### Desired Roll

$$
\phi_d=-K_9(y-y_d)-K_{10}\dot{y}
$$

Similarly, the first term,

$$
-K_9(y-y_d)
$$

depends on the error between the current and desired $y$ position.

The second term,

$$
-K_{10}\dot{y}
$$

## 4.5 Quadrotor Attitude Control

The quadrotor attitude controller uses P/PD control.

### Roll Control

The control input is:

$$
U_2=-K_1(\phi-\phi_d)-K_2\dot{\phi}
$$

where:

- $\phi$ = roll angle
- $\phi_d$ = desired roll angle
- $K_1$ = proportional gain
- $K_2$ = derivative gain

### Pitch Control

The control input is:

$$
U_3=-K_3(\theta-\theta_d)-K_4\dot{\theta}
$$

where:

- $\theta$ = pitch angle
- $\theta_d$ = desired pitch angle
- $K_3$ = proportional gain
- $K_4$ = derivative gain

### Yaw Control

The project also defines the yaw controller using:

$$
U_4=-K_5(\psi-\psi_d)-K_6\dot{\psi}
$$

The main numerical comparison focuses on roll and pitch.

---

## 4.6 Controller Gains

The attitude-control gains used in the project are:

| Controller | Gain | Value |
|---|---|---:|
| Roll P | K1 | 0.625 |
| Roll D | K2 | 0.170 |
| Pitch P | K3 | 0.810 |
| Pitch D | K4 | 0.340 |
| Yaw P | K5 | 0.370 |
| Yaw D | K6 | 0.100 |

The roll and pitch gains are based on the gains reported in the paper's attitude-control experiment.

---

## 4.7 P and PD Control Simulation

A 10-second numerical simulation is performed for both roll and pitch.

### Roll

The simulation compares:

```text
P Controller
     vs
PD Controller
```

The roll disturbance is represented numerically using sinusoidal disturbance terms.

### Pitch

Similarly:

```text
P Controller
     vs
PD Controller
```

The pitch simulation also uses sinusoidal disturbance terms.

The purpose is to demonstrate the effect of the derivative term on attitude stabilization.

---

## 4.8 2-DOF Camera Stabilizer

The camera stabilizer consists of two independently controlled axes:

```text
             Camera Stabilizer
                  │
          ┌───────┴───────┐
          ▼               ▼
       Roll Axis       Pitch Axis
          │               │
       Servo 1         Servo 2
```

The yaw degree of freedom is not included in the stabilizer. The reference paper explains that yaw motion of the stabilizer can be replaced by the yaw control of the main quadrotor body.

This reduces:

- Mechanism weight
- Energy consumption
- Mechanical complexity

---

## 4.9 Camera PID Control

The camera roll error is:

$$
e_{\phi C}=\phi_{Cd}-\phi_C
$$

The camera pitch error is:

$$
e_{\theta C}=\theta_{Cd}-\theta_C
$$

The roll control input is:

$$U_{\phi C} = K_{11}e_{\phi C} + K_{12}\int e_{\phi C}dt + K_{13}\dot{e}_{\phi C}$$

The pitch control input is:

$$U_{\theta C} = K_{14}e_{\theta C} + K_{15}\int e_{\theta C}dt + K_{16}\dot{e}_{\theta C}$$

---

## 4.10 Camera PID Gains

The gains used in the simulation are:

| Axis | P | I | D |
|---|---:|---:|---:|
| Camera Roll | 170.0 | 2.0 | 0.1 |
| Camera Pitch | 170.0 | 2.0 | 0.1 |

These are the gain values reported for the camera stabilization experiment in the reference paper.

---

## 4.11 Camera Disturbance

The camera stabilizer is tested against a simulated manipulator/body inclination of approximately ±45°.

The disturbance profile is:

```text
0 s → 7 s       : 0° → +45°
7 s → 21 s      : +45° → -45°
21 s → 28 s     : -45° → 0°
```

The purpose is to test whether the camera can remain close to its desired zero-degree attitude while the base changes orientation.

---

## 4.12 Numerical Simulation

The `main.m` script performs the complete numerical simulation.

The process is:

```text
Load Parameters
      ↓
Tether Position Calculation
      ↓
Roll P Simulation
      ↓
Roll PD Simulation
      ↓
Pitch P Simulation
      ↓
Pitch PD Simulation
      ↓
Camera Roll PID Simulation
      ↓
Camera Pitch PID Simulation
      ↓
Generate Plots
```

The numerical integration is performed using a time-stepping approach for angular acceleration, angular velocity, and angle.

---

# 5. Experimental Setup

The numerical experiments reproduce the control conditions represented in the reference paper.

### Attitude Control

- Simulation duration: 10 seconds
- Roll: P vs PD
- Pitch: P vs PD
- Desired roll: 0°
- Desired pitch: 0°

### Camera Stabilization

- Simulation duration: 28 seconds
- Base/manipulator inclination: approximately ±45°
- Camera desired roll: 0°
- Camera desired pitch: 0°

The paper experimentally suspended the quadrotor for attitude tests and used a robot manipulator to incline the camera stabilizer by ±45° for camera stabilization tests.

---

# 6. Simulation Results

## 6.1 Roll Attitude Control

The project generates a comparison between:

```text
(a) Roll control using P controller
(b) Roll control using PD controller
```

The reference paper reports that the experimental roll error was reduced from a maximum of approximately 12.0° with P control to approximately 8.7° with PD control.

The generated MATLAB plot is used to visualize the corresponding simulated response.

---
<p align="center"> <img src="pitch_P_vs_PD.jpeg" alt="Pitch angle response: P control vs PD control" width="650"> </p>

## 6.2 Pitch Attitude Control

The project generates:

```text
(a) Pitch control using P controller
(b) Pitch control using PD controller
```

The reference paper reports a maximum pitch error of approximately -10.9° with P control and approximately 4.1° with PD control.

This demonstrates the improvement obtained by adding the derivative term.

---
<p align="center"> <img src="roll_P_vs_PD.jpeg" alt="Roll angle response: P control vs PD control" width="650"> </p>

## 6.3 Camera Roll Stabilization

The simulation compares:

```text
Camera roll angle
       vs
Manipulator/base angle
```

The base is moved through approximately +45° and -45°.

The objective is to maintain the camera roll close to 0° while the base angle changes.

---
<p align="center"> <img src="camera_roll_stabilization.jpeg" alt="Camera roll stabilization vs base angle" width="600"> </p>

## 6.4 Camera Pitch Stabilization

The same procedure is applied to camera pitch.

The simulation compares:

```text
Camera pitch angle
       vs
Manipulator/base angle
```

The PID controller attempts to compensate for the base motion and maintain the camera attitude near the desired zero-degree reference.

---
<p align="center"> <img src="camera_pitch_stabilization.jpeg" alt="Camera pitch stabilization vs base angle" width="600"> </p>



# 7. Performance Analysis

The project evaluates the control system using the following observations:

### Attitude Control

- P control provides proportional correction based on attitude error.
- PD control additionally uses angular velocity feedback.
- The derivative term improves damping and reduces attitude error in the reference experiment.

### Camera Stabilization

- The 2-DOF stabilizer independently controls roll and pitch.
- The PID controller compensates for base/manipulator inclination.
- The camera is intended to remain close to the desired zero-degree attitude.

### Tether Position

- Tether inclination and length are used to calculate the quadrotor position.
- The method does not require GPS position information for this calculation.

---

# 8. Project Structure

The main project structure is:

```text
Tethered_Quadrotor_Project/
├── assets/
│   └── amrita_logo.jpg
├── README.md
├── main.m
├── params.m
├── tether_pos_func.m
├── build_simulink_model.m
└── slprj/
    └── sim/
        └── varcache/
```

### File Description

| File | Purpose |
|---|---|
| `main.m` | Main numerical simulation and plotting script |
| `params.m` | Physical parameters and controller gains |
| `tether_pos_func.m` | Tether-based position calculation |
| `build_simulink_model.m` | Automatically builds the Simulink model |
| `slprj/` | MATLAB/Simulink generated simulation files |
| `README.md` | Project documentation |

---

# 9. Conclusion

This project implements a MATLAB/Simulink control-oriented model of a camera-mounted tethered quadrotor for infrastructure inspection.

The system combines tether-based position sensing, quadrotor roll/pitch attitude control, P and PD controller comparison, and a 2-DOF PID camera stabilizer.

The MATLAB implementation provides numerical simulations and plots, while the Simulink implementation provides a block-based representation of the major control subsystems.

The reference paper experimentally demonstrated improved roll and pitch attitude control using PD control compared with P control, and demonstrated that the 2-DOF camera stabilizer could maintain camera roll and pitch close to 0° under ±45° base inclination.

Overall, the project provides a simulation framework for studying attitude stabilization and camera stabilization of a tethered quadrotor intended for infrastructure inspection.

---

# 10. References

1. Keigo Watanabe, Nao Moritoki, and Isaku Nagai, **"Attitude Control of a Camera Mounted-type Tethered Quadrotor for Infrastructure Inspection,"** IEEE conference paper.

2. Watanabe et al., **"Development of a Camera-mounted Tethered Quadrotor for Inspecting Infrastructures,"** IECON 2016.

3. Ouchi et al., **"Position Control of an X4-Flyer Using a Tether,"** International Journal of Smart Material and Mechatronics, 2014.

4. S. Lupashin and R. D'Andrea, **"Stabilization of a Flying Vehicle on a Taut Tether using Inertial Sensing,"** IROS, 2003.

5. S. Bouabdallah, P. Murrieri, and R. Siegwart, **"Towards Autonomous Indoor Micro VTOL,"** Autonomous Robots, 2005.
