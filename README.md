# 🛰️ Orbital Kalman Tracker: Real-Time Stochastic Estimation

A simple but high-performance implementation of a **Linear Kalman Filter (LKF)** designed for real-time orbital trajectory estimation. This project simulates a Satellite Tracking System in Low Earth Orbit (LEO), filtering Gaussian noise from sensor measurements to predict precise positioning and velocity.

> Note: This project was originally developed on a smartphone using Pydroid. Due to this, some features may not function correctly on desktop screens.

### Free Sample (Pydroid version)
<img src="animated/orbital_tracking.gif" width="350"> 

## 🔬 Engineering Overview
Modern orbital systems deal with high-frequency sensor noise and signal dropouts. This project implements a **Constant Velocity (CV) Model** to maintain state estimation even when sensor data is unreliable.

### Technical Stack
- **State Vector ($x$):** 4-dimensional $[pos_x, pos_y, vel_x, vel_y]$
- **Mathematics:** Discrete-time state-space representation.
- **Buffer Architecture:** Implemented via `collections.deque` for $O(1)$ real-time data feeding at 25Hz.
- **Engine:** Python / NumPy (optimized for matrix arithmetic).

## 🧮 How it Works
The filter operates in a recursive **Predict-Correct** loop:

1. **Prediction:** Projects the current state and error covariance forward in time using the Transition Matrix ($F$).
2. **Measurement:** Simulates a noisy orbital position ($z$) from a ground-truth elliptical trajectory.
3. **Correction:** Calculates the **Kalman Gain ($K$)** to weight the uncertainty between the physical model and the sensor input, updating the state estimate.

> "The filter is tuned to prioritize the physical model ($Q$) over high-variance sensor noise ($R$), resulting in a smooth, high-fidelity orbital path."

## 🚀 Key Features
- **Real-Time Telemetry:** Live calculation of Altitude and Velocity magnitude.
- **Circular Buffers:** Efficient memory management for long-duration tracking simulations.
- **Dynamic Visualization:** Custom Dark-Mode HUD (Heads-Up Display) styled for Mission Control environments.

## 🛠️ How to Run
```bash
# Clone the repository
git clone [https://github.com/your-username/orbital-kalman-tracker.git](https://github.com/your-username/orbital-kalman-tracker.git)

# Install dependencies
pip install numpy matplotlib

# Execute the simulation
python main.py
