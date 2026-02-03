# Design and Implementation of a PID Controller for an RC Circuit

**Course:** CIE 318: Control Systems Project SP25  
**Institution:** University of Science and Technology in Zewail City  
**Date:** May 4, 2025

## 📌 Project Overview
This project presents a comprehensive analysis and control design for an electric circuit modeled using state-space and transfer function approaches. The primary goal is to design a feedback control system for a specific RC circuit (comprising two capacitors and resistors) to regulate the voltage across one of the capacitors.

The project moves from theoretical modeling to software simulation using MATLAB/Simulink and LTSpice, concluding with a hardware implementation using both analog operational amplifiers and an Arduino microcontroller to validate the control design.

## 👥 Team Members
* **Mohammed Ali** (202200594)
* **Abdulrahman Magdy** (202200341)
* **Ahmed Amgad** (202200393)
* **Alaa Sabry** (202200331)

**Supervisor:** Dr. Abd-Elshafaei

## 🎯 Objectives
The control system is designed to meet the following performance metrics:
* **Settling Time:** Approximately 6.5 seconds.
* **Maximum Overshoot:** Less than 8%.
* **Steady-State Error:** Reduction/Elimination using integral action (PID).

## ⚙️ System Model
The circuit consists of two capacitors ($C_1, C_2 = 10\mu F$) and resistors ($R_1, R_2 = 100 k\Omega$).

* **Transfer Function:**
    $$G(s) = \frac{1}{s^2 + 3s + 1}$$
    Derived from the state-space representation where state variables are the voltages across the capacitors.
* **Open-Loop Characteristics:**
    * DC Gain: 1
    * Natural Frequency: 1 rad/s
    * Damping Ratio: 1.5 (Overdamped).

## 🚀 Methodology

### 1. Theoretical Modeling & Simulation
* **State-Space Analysis:** Derived system matrices ($A, B, C, D$) to represent circuit dynamics.
* **MATLAB `sisotool`:** Used to investigate proportional gain controllers and analyze root locus/bode plots.
* **PID Tuning:** The PID controller was tuned using **Simulink** to meet transient response specifications.
    * *Final PID Parameters:* $K_p = 2.3$, $K_i = 0.98$, $K_d = 0.3$.

### 2. Hardware Implementation
The controller was implemented physically using two distinct methods to validate simulation results:
* **Analog Implementation:** Constructed using Operational Amplifiers (Op Amps) to replicate the PID logic using analog components. Circuit design and simulation were performed in **LTSpice**.
* **Digital Implementation:** Implemented using an **Arduino Uno** microcontroller. The control logic was deployed directly from a Simulink model to the hardware.

## 🛠️ Tools & Technologies
* **MATLAB & Simulink:** For transfer function derivation, PID tuning, and Arduino interfacing.
* **LTSpice:** For analog circuit design and realistic simulation including non-ideal component behavior.
* **Arduino IDE/Hardware:** For digital control implementation.
* **Hardware Components:** Resistors ($100k\Omega$), Capacitors ($10\mu F$), Op Amps (LM741), Arduino Uno Board.

## 📊 Key Results
* **Simulation vs. Reality:** The Arduino implementation closely matched the LTSpice simulation results, validating the hardware performance.
* **Real-world Constraints:** LTSpice simulations proved more accurate than ideal Simulink models for predicting hardware behavior due to the inclusion of parasitic effects and non-ideal component behaviors.
* **Performance:** The final design successfully achieved the target settling time and overshoot specifications.

## 📂 Repository Structure
├── /simulations │ ├── matlab_code/ # MATLAB scripts for State-Space & TF derivation │ ├── simulink_models/ # PID tuning and Arduino control models │ └── ltspice_circuits/ # Analog controller schematics and simulation files ├── /docs │ ├── report.pdf # Full project report │ └── figures/ # System response plots and circuit diagrams └── README.md # Project documentation


## 📚 References
* The MathWorks, Inc., *Control System Toolbox User’s Guide*, 2024.
* K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*, 1995.
* Texas Instruments, *LM741 Operational Amplifier Datasheet*, 2015.
