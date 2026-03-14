import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class OrbitalTracker:
    def __init__(self, buffer_size=50):
        # Simulation Constants
        self.dt = 0.15
        self.earth_radius = 6.0
        
        # State Vector [pos_x, pos_y, vel_x, vel_y]
        self.x = np.array([12.0, 0.0, 0.0, 5.0], dtype=float)
        self.P = np.eye(4) * 10.0  # State Covariance
        
        # Kalman Matrices
        self.F = np.array([[1, 0, self.dt, 0], 
                           [0, 1, 0, self.dt], 
                           [0, 0, 1, 0], 
                           [0, 0, 0, 1]]) # Transition Matrix
        self.H = np.array([[1, 0, 0, 0], 
                           [0, 1, 0, 0]]) # Measurement Matrix
        self.Q = np.eye(4) * 0.05      # Process Noise
        self.R = np.eye(2) * 3.0       # Sensor Noise
        
        # Circular Buffers for Real-Time Plotting
        self.trail_x = deque(maxlen=buffer_size)
        self.trail_y = deque(maxlen=buffer_size)
        self.meas_x = deque(maxlen=buffer_size)
        self.meas_y = deque(maxlen=buffer_size)
        
        self.time_step = 0

    def predict_and_update(self):
        """Core Kalman Filter Loop (Prediction + Correction)"""
        # 1. Prediction Phase
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # 2. Simulate Noisy Measurement (Ground Truth + Gaussian Noise)
        # Using an ellipse as the ground truth orbit
        true_x = 12 * np.cos(self.time_step * 0.1)
        true_y = 16 * np.sin(self.time_step * 0.1)
        z = np.array([true_x, true_y]) + np.random.normal(0, 0.8, 2)
        
        # 3. Update Phase (Correction)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S) # Kalman Gain
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
        # Update Buffers
        self.trail_x.append(self.x[0])
        self.trail_y.append(self.x[1])
        self.meas_x.append(z[0])
        self.meas_y.append(z[1])
        
        self.time_step += 1
        return z

# --- Graphics Configuration ---
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(8, 8))
fig.patch.set_facecolor('#050505')
ax.set_facecolor('#050505')

tracker = OrbitalTracker(buffer_size=60)

# Visual Assets
earth = plt.Circle((0, 0), tracker.earth_radius, color='#001a33', ec='#00ffff', lw=2, alpha=0.8)
ax.add_patch(earth)

line_meas, = ax.plot([], [], 'o', color='#ff00ff', alpha=0.3, markersize=2)
line_kalman, = ax.plot([], [], color='#00ffff', lw=2, alpha=0.9)
point_sat, = ax.plot([], [], 'wo', markersize=8, markeredgecolor='#00ffff')

ax.set_xlim(-25, 25)
ax.set_ylim(-25, 25)
ax.axis('off')

hud = ax.text(-23, 22, "", color='#00ffff', family='monospace', fontsize=9, 
              bbox=dict(facecolor='black', alpha=0.7, edgecolor='#00ffff'))

def animate(i):
    z = tracker.predict_and_update()
    
    # Update Graphic Data from Deque
    line_meas.set_data(list(tracker.meas_x), list(tracker.meas_y))
    line_kalman.set_data(list(tracker.trail_x), list(tracker.trail_y))
    point_sat.set_data([tracker.x[0]], [tracker.x[1]])
    
    # Telemetry Calculation
    alt = np.linalg.norm(tracker.x[:2]) - tracker.earth_radius
    vel = np.linalg.norm(tracker.x[2:])
    
    telemetry = (
        f"🛰️ ORBITAL TELEMETRY\n"
        f"--------------------\n"
        f"ALTITUDE: {alt:.2f} units\n"
        f"VELOCITY: {vel:.2f} u/s\n"
        f"SYNC:     LOCKED\n"
        f"T-PLUS:   {tracker.time_step}s"
    )
    hud.set_text(telemetry)
    return line_meas, line_kalman, point_sat, hud

# Frame-less animation (runs indefinitely at ~25Hz)
ani = FuncAnimation(fig, animate, interval=40, blit=True, cache_frame_data=False)

plt.tight_layout()
plt.show()