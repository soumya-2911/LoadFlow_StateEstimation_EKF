# Power System Dynamic State Estimation with EKF

This repository implements **Dynamic State Estimation (DSE)** for power
systems using the **Extended Kalman Filter (EKF)**.\
It provides both **static power flow analysis** (Newton--Raphson method)
and **dynamic simulations** with disturbances and harmonics for IEEE
test systems (3-bus and 14-bus).

------------------------------------------------------------------------

## 📂 Repository Structure

-   **`dynamic_state_estimator_nbus.py`**
    -   Implements the `DynamicStateEstimator_nBus` class for
        **multi-bus EKF-based state estimation**.
    -   Models generator dynamics (rotor angle, speed, inertia, damping,
        internal voltages).
    -   Defines measurement model (bus voltages, angles, generator power
        outputs).
    -   Provides Jacobians for EKF prediction and update.
    -   Includes **IEEE 3-bus and 14-bus system data creation
        functions**.
    -   Interfaces with Newton-Raphson power flow results for
        initialization.
-   **`Newton_Raphson.py`**
    -   Implements the **Newton-Raphson power flow solver** in polar
        coordinates.
    -   Reads bus and line data from JSON files in `state_est_utility/`.
    -   Outputs bus voltages, angles, power injections, and updates
        system state.
    -   Provides a clean object-oriented interface (`Newton_Raphson`
        class).
    -   Prints formatted bus results table for clarity.
-   **`run_dynamic_EKF.py`**
    -   Provides a **simulation driver** for running dynamic state
        estimation with disturbances.
    -   Implements multiple disturbance scenarios:
        -   Step load changes
        -   Generator trips
        -   Oscillatory load
        -   Three-phase fault
        -   Harmonic distortions:
            -   Nonlinear load harmonics (3rd, 5th, 7th)
            -   VFD harmonics (5th, 7th, 11th, 13th)
            -   Arc furnace harmonics
            -   LED lighting harmonics
            -   Capacitor switching resonance
            -   Thyristor rectifier harmonics
            -   Wind farm harmonics
    -   Records and plots:
        -   Rotor angle and frequency
        -   Bus voltages
        -   Mechanical power
    -   Uses `VoltageWaveformVisualizer` for realistic AC waveform
        visualization.
-   **`VoltageWaveformVisualizer.py`**
    -   Generates **synthetic AC voltage waveforms** from estimated
        states.
    -   Converts voltage magnitudes and angles into time-domain
        sinusoidal waveforms.
    -   Plots **2D voltage waveforms** with envelopes to show dynamic
        variations.
-   **`state_estimator_static.ipynb`**
    -   Jupyter notebook for **static state estimation experiments**.
    -   Provides demonstrations of power flow, initialization, and
        simple EKF runs.

------------------------------------------------------------------------

## ⚡ Features

-   **Static and Dynamic Estimation**
    -   Newton--Raphson static solution for initialization.
    -   Extended Kalman Filter for dynamic tracking of generator states.
-   **Disturbance Modeling**
    -   Step, pulse, oscillatory, fault, and harmonics.
    -   Captures realistic power quality issues (harmonics, capacitor
        switching, arc furnace behavior).
-   **Visualization**
    -   Bus voltages, rotor angles, and frequency responses.
    -   Time-domain AC voltage waveforms with harmonics and transients.

------------------------------------------------------------------------

## 🚀 How to Run

1.  Clone the repository and ensure dependencies are installed:

    ``` bash
    pip install numpy scipy matplotlib pandas
    ```

2.  Prepare **bus and line data** in `state_est_utility/`:

    -   `bus.json`
    -   `data_line.json`

3.  Run a dynamic simulation:

    ``` bash
    python run_dynamic_EKF.py
    ```

4.  Example: Run with different disturbance

    ``` python
    sim = run_dynamic_EKF()
    results = sim.run(T=3.0, dist_type='three_phase_fault')
    ```

------------------------------------------------------------------------

## 📊 Example Outputs

-   Rotor angle and frequency deviation plots
-   Bus voltage magnitude and phase plots
-   AC waveform plots with harmonic distortions
-   Event log of disturbances applied

------------------------------------------------------------------------

## 🔧 Applications

-   Power system **dynamic assessment for Machine Learning Applications**
-   **State estimation under harmonics** and nonlinear loads
-   Educational use for power system dynamics and estimation

------------------------------------------------------------------------

## 👨‍💻 Authors

Soumya Ranjan Das

contact- soumyaranjan07new@gmail.com
