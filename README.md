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

MATLAB is used to simulate the control system and generate the corresponding response plots. In addition, a Simulink model is developed to represent the major parts of the system, including tether-based position sensing, quadrotor attitude dynamics, attitude control, and camera stabilization. The simulation helps to understand how the quadrotor and camera respond to disturbances and how the controllers maintain the desired orientation.

---

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

### 2.6 MATLAB Simulation

Implement the mathematical models using numerical integration and generate the required plots.

### 2.7 Simulink Model

Automatically construct an interactive Simulink model containing the major subsystems of the proposed control architecture.

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

- MATLAB
- MATLAB Simulink
- Numerical ODE integration
- MATLAB plotting functions

The MATLAB script performs the numerical simulations directly, while `build_simulink_model.m` creates the corresponding Simulink model.

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
l^2\cos^2\alpha\cos^2\beta
}{
\cos^2\alpha+\cos^2\beta-\cos^2\alpha\cos^2\beta
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

$$U_{\phi C} = K_{11}e_{\phi C} + K_{12}\int e_{\phi C}\,dt + K_{13}\dot{e}_{\phi C}$$

The pitch control input is:

$$U_{\theta C} = K_{14}e_{\theta C} + K_{15}\int e_{\theta C}\,dt + K_{16}\dot{e}_{\theta C}$$

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
