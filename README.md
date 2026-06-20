# DC Motor Speed Control System via SLDRT

This repository contains a complete MATLAB & Simulink implementation for the experimental system identification and speed control design of a permanent-magnet DC motor. The project bridges the gap between empirical physical modeling and real-time digital controller execution using **Simulink Desktop Real-Time (SLDRT)**.

---

## 📌 Project Overview

The project is structured into two main phases based on rigorous control engineering lab practices:

1. **System Identification (Phase 1):** Real-time parameters of a physical DC motor setup (including moment of inertia $J$ and non-linear friction coefficients $\tau_{F0}, \alpha_F$) were identified using coast-down test data and data acquisition (`CS_V2`).
2. **Controller Design & Real-Time Implementation (Phase 2):** A cascaded control loop configuration (utilizing an inner current loop and an outer speed loop) was modeled in Simulink and deployed in real-time to track aggressive step-trajectories under varying load disturbances (`CS_V5`).

---

## 🛠️ System Architecture & Controller Design

The control scheme features an industrial cascaded loop architecture to safely manage physical armature current limits ($\pm 10\text{ A}$) and voltage constraints ($\pm 42\text{ V}$) while maximizing speed tracking performance.

### 1. Plant Dynamics & Mathematical Modeling
The underlying continuous-time dynamic behavior of the permanent-magnet DC motor is governed by the following electromechanical differential equations:

$$R_{A}\cdot i_{A}(t) + L_{A}\frac{di_{A}(t)}{dt} = u_{A}(t) - e_{M}(t)$$
$$J\frac{d\omega(t)}{dt} = \tau_{M}(t) - \tau_{F}(t) - \tau_{L}(t)$$
$$e_{M}(t) = k_{G}\cdot\omega(t), \quad \tau_{M}(t) = k_{G}\cdot i_{A}(t)$$

Where friction torque $\tau_{F}$ is captured as $\tau_{F} = \tau_{F0}\text{sgn}(\omega) + \alpha_{F}\omega$. Sensor noise is suppressed using low-pass filters with time constants $T_{Fn} = 0.02\text{ s}$ for speed and $T_{Fi} = 0.002\text{ s}$ for current.

### 2. Inner Loop: Current Controller Design (Modulus Optimum)
The inner current loop tracks the required armature current ($i_{A\_sp}$) by manipulating the power amplifier voltage input ($u_A$). A PI controller is designated as:

$$G_{Ri}(s) = K_{Ri}\left(1 + \frac{1}{T_{Ii}s}\right)$$

During the design, the induced EMF ($e_M$) is treated as an unknown disturbance due to slower mechanical dynamics. The plant is extended to account for the digital control computer operating at a sample time of $T_S = 0.001\text{ s}$. The dead-time of the sample-and-hold circuit is modeled as a first-order lag ($0.5 T_S s + 1$). 

Small parasitic time constants are combined into an equivalent time constant $T_{\sigma} = T_{Fi} + 0.5T_{S}$. Applying the **Modulus Optimum (Betragsoptimum)** criteria to optimize the closed-loop magnitude response, the controller parameters are calculated as:
* **Integral Time Constant ($T_{Ii}$):** Compensates the dominant electrical plant lag:  
  $$T_{Ii} = T_1 = \frac{L_A}{R_A}$$
* **Proportional Gain ($K_{Ri}$):** $$K_{Ri} = \frac{T_{Ii}}{2 K_S T_{\sigma}} = \frac{L_A}{2T_{Fi} + T_S}$$

### 3. Outer Loop: Speed Controller Design (Symmetrical Optimum)
The outer speed loop regulates the rotor velocity ($n$) by producing a dynamic current reference command ($i_{A\_sp}$) for the inner controller. A PI controller is defined as:

$$G_{Rn}(s) = K_{Rn}\left(1 + \frac{1}{T_{In}s}\right)$$

The speed loop plant model combines the closed-loop tracking transfer function of the inner current loop ($G_i(s)$), the rigid mechanical rotatory system components ($G_m(s)$), and the low-pass speed signal filter ($G_{Fn}(s)$). 

The controller parameters ($K_{Rn}$ and $T_{In}$) are determined by applying the **Symmetrical Optimum** tuning method using MATLAB’s `sisotool` (Control System Designer app) to satisfy a target phase margin specification of **$\varphi_{M} = 37^{\circ}$**. This optimization maximizes phase margin at the crossover frequency, ensuring excellent robust disturbance rejection when external loads are sudden or unpredictable.

### 4. Integrator Anti-Windup Configuration
To prevent integrator windup and subsequent control saturation oscillations when operating against physical actuator limit bounds, both controllers incorporate dedicated back-calculation anti-windup loops. The anti-windup tracking gains are defined relative to the controller gains:
* **Current Anti-Windup Gain:** $K_{awi} = K_{Ri}$
* **Speed Anti-Windup Gain:** $K_{awn} = 50 \cdot K_{Rn}$

---

## 📈 Performance & Results

The tracking capability of the designed cascaded controller was validated using two distinct operational step trajectories:


### 1. Low-Speed Trajectory ($50\text{ rpm}$)
* **Characteristics:** Rapid, heavily damped transient response settling precisely at the reference target with negligible steady-state error.
* **Transient Dynamics:** The control input voltage smoothly ramps up to overcome initial static friction and stabilizes perfectly without major overshoot.
<img width="514" height="316" alt="image" src="https://github.com/user-attachments/assets/05ed45ef-84bf-44c8-8dee-ad67cf125642" />

### 2. High-Speed Trajectory with Load Disturbance ($2000\text{ rpm}$)
* **Characteristics:** The motor rapidly accelerates up to $2000\text{ rpm}$ within a minimal settling time window.
* **Disturbance Rejection:** After approximately 6 seconds, an external resistive load torque was manually engaged. The system demonstrates excellent robust control authority—recovering back to the $2000\text{ rpm}$ steady-state target promptly after a minor, temporary speed drop.
<img width="490" height="315" alt="image" src="https://github.com/user-attachments/assets/d0e771db-d6ef-4e97-86dd-603ce51592a3" />

---

## 📂 Repository Structure

* **`DCMotor_controlsim_older.slx`** — The primary Simulink block diagram containing the cascaded controller architecture, saturation blocks, anti-windup subsystems, real-time interface blocks (National Instruments PCIe-6321 auto setup mappings), and filtering blocks for sensor noise suppression.
* **`Motor_control_script.m`** — The initialization MATLAB script containing the identified physical plant parameters ($J, R_A, L_A, K_b$) and computed controller gains ($K_p, T_i$). **Run this file first before opening the Simulink model.**

---

## 🚀 How to Run the Project

### Prerequisites
* MATLAB & Simulink (with **Simulink Coder** and **Simulink Desktop Real-Time** toolboxes installed).
* A supported Data Acquisition Board configuration (default configured for NI PCIe-6321 via BNC-2110 interface box).

### Execution Steps
1. Clone this repository to your local machine:
   ```bash
   git clone [https://github.com/your-username/DCMotor_controlsim_older.git](https://github.com/your-username/dc-motor-speed-control.git)
