import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj

# ─────────────────────────────────────────
# LOAD DATA
# ─────────────────────────────────────────
print("Loading data...")

ekf = pd.read_csv('/home/rian/ros2_ws/src/ekf_gps_imu/data/ekf_output.csv')
gps = pd.read_csv('/home/rian/ros2_ws/src/ekf_gps_imu/data/gpstopic.csv')
imu = pd.read_csv('/home/rian/ros2_ws/src/ekf_gps_imu/data/imutopic.csv')

print(f"EKF rows:  {len(ekf)}")
print(f"GPS rows:  {len(gps)}")
print(f"IMU rows:  {len(imu)}")

# ─────────────────────────────────────────
# CONVERT GPS LAT/LON TO UTM METERS
# ─────────────────────────────────────────
proj = Proj(proj='utm', zone=19, ellps='WGS84')
gps_x, gps_y = proj(gps['longitude.data'].values, gps['latitude.data'].values)

# Normalize to start at 0,0
origin_x = gps_x[0]
origin_y = gps_y[0]

gps_x = gps_x - origin_x
gps_y = gps_y - origin_y
ekf_x = ekf['ekf_x'].values - origin_x
ekf_y = ekf['ekf_y'].values - origin_y

# ─────────────────────────────────────────
# PLOT 1 — GPS vs EKF Trajectory
# ─────────────────────────────────────────
max_range = np.max(np.sqrt(gps_x**2 + gps_y**2)) * 2
mask = (np.abs(ekf_x) < max_range) & (np.abs(ekf_y) < max_range)
ekf_x_clean = ekf_x[mask]
ekf_y_clean = ekf_y[mask]

plt.figure(figsize=(12, 8))
plt.scatter(gps_x, gps_y, c='red', s=10, alpha=0.5, label='Raw GPS (noisy)')
plt.plot(ekf_x_clean, ekf_y_clean, 'b-', linewidth=1.5, label='EKF Filtered Path')
plt.plot(ekf_x_clean[0], ekf_y_clean[0], 'go', markersize=10, label='Start')
plt.plot(ekf_x_clean[-1], ekf_y_clean[-1], 'rs', markersize=10, label='End')

plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('EKF GPS/IMU Fusion — Boston Driving\nRaw GPS vs Filtered Path')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('/home/rian/ros2_ws/src/ekf_gps_imu/analysis/plot1_trajectory.png', dpi=150)
plt.show()
print("Plot 1 saved!")

# ─────────────────────────────────────────
# PLOT 2 — Velocity over time
# ─────────────────────────────────────────
plt.figure(figsize=(12, 5))
time = np.arange(len(ekf)) * 0.025

plt.subplot(1, 2, 1)
plt.plot(time, ekf['vx'].values, 'b-', linewidth=1, label='vx')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('X Velocity over Time')
plt.grid(True)
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(time, ekf['vy'].values, 'r-', linewidth=1, label='vy')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Y Velocity over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig('/home/rian/ros2_ws/src/ekf_gps_imu/analysis/plot2_velocity.png', dpi=150)
plt.show()
print("Plot 2 saved!")

# ─────────────────────────────────────────
# PLOT 3 — Yaw over time
# ─────────────────────────────────────────
plt.figure(figsize=(12, 5))

gz = imu['angular_velocity.z'].values
dt = 0.025
raw_yaw = np.cumsum(gz * dt)
time_imu = np.arange(len(imu)) * dt
time_ekf = np.arange(len(ekf)) * dt

plt.plot(time_imu, np.degrees(raw_yaw), 'r-',
         linewidth=1, alpha=0.7, label='Raw Gyroscope Yaw (drifts)')
plt.plot(time_ekf, np.degrees(ekf['yaw'].values), 'b-',
         linewidth=1.5, label='EKF Corrected Yaw')

plt.xlabel('Time (s)')
plt.ylabel('Yaw (degrees)')
plt.title('Yaw Estimation — Raw Gyroscope vs EKF Corrected')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('/home/rian/ros2_ws/src/ekf_gps_imu/analysis/plot3_yaw.png', dpi=150)
plt.show()
print("Plot 3 saved!")

# ─────────────────────────────────────────
# PLOT 4 — Magnetometer Calibration
# ─────────────────────────────────────────
mag_x = imu['magnetic_field.x'].values
mag_y = imu['magnetic_field.y'].values

# Remove outliers using IQR
def remove_outliers(x, y):
    q1x, q3x = np.percentile(x, 25), np.percentile(x, 75)
    q1y, q3y = np.percentile(y, 25), np.percentile(y, 75)
    iqr_x = q3x - q1x
    iqr_y = q3y - q1y
    mask = ((x > q1x - 1.5*iqr_x) & (x < q3x + 1.5*iqr_x) &
            (y > q1y - 1.5*iqr_y) & (y < q3y + 1.5*iqr_y))
    return x[mask], y[mask]

mag_x_clean, mag_y_clean = remove_outliers(mag_x, mag_y)

# Hard iron correction
hard_iron_x = np.mean(mag_x_clean)
hard_iron_y = np.mean(mag_y_clean)
mag_x_hard = mag_x_clean - hard_iron_x
mag_y_hard = mag_y_clean - hard_iron_y

# Soft iron correction
scale_x = (np.max(mag_x_hard) - np.min(mag_x_hard)) / 2
scale_y = (np.max(mag_y_hard) - np.min(mag_y_hard)) / 2
avg_scale = (scale_x + scale_y) / 2
mag_x_soft = mag_x_hard * (avg_scale / scale_x)
mag_y_soft = mag_y_hard * (avg_scale / scale_y)

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

axes[0].scatter(mag_x_clean, mag_y_clean, s=1, alpha=0.3, c='red')
axes[0].set_title('Raw Magnetometer\n(shifted ellipse)')
axes[0].set_xlabel('Mag X')
axes[0].set_ylabel('Mag Y')
axes[0].grid(True)
axes[0].set_aspect('equal')

axes[1].scatter(mag_x_hard, mag_y_hard, s=1, alpha=0.3, c='orange')
axes[1].set_title('After Hard Iron\n(centered but still ellipse)')
axes[1].set_xlabel('Mag X')
axes[1].set_ylabel('Mag Y')
axes[1].grid(True)
axes[1].set_aspect('equal')

axes[2].scatter(mag_x_soft, mag_y_soft, s=1, alpha=0.3, c='green')
axes[2].set_title('After Soft Iron\n(circle at origin)')
axes[2].set_xlabel('Mag X')
axes[2].set_ylabel('Mag Y')
axes[2].grid(True)
axes[2].set_aspect('equal')

plt.suptitle('Magnetometer Calibration — Hard Iron + Soft Iron Correction',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig('/home/rian/ros2_ws/src/ekf_gps_imu/analysis/plot4_magnetometer.png', dpi=150)
plt.show()
print("Plot 4 saved!")

# ─────────────────────────────────────────
# PLOT 5 — Stationary Analysis (FIXED)
# ─────────────────────────────────────────

# Find stationary rows — within 1m of starting point
dist_from_start = np.sqrt(gps_x**2 + gps_y**2)
stationary_mask = dist_from_start < 1.0
stationary_indices = np.where(stationary_mask)[0]

print(f"Found {len(stationary_indices)} stationary GPS readings (within 1m of start)")

# Use stationary rows
stat_x = gps_x[stationary_indices]
stat_y = gps_y[stationary_indices]

# Center around mean
mean_x = np.mean(stat_x)
mean_y = np.mean(stat_y)
stat_x = stat_x - mean_x
stat_y = stat_y - mean_y

var_x = np.var(stat_x)
var_y = np.var(stat_y)
std_x = np.std(stat_x)
std_y = np.std(stat_y)

print(f"GPS noise std: x={std_x:.3f}m, y={std_y:.3f}m")

plt.figure(figsize=(8, 8))
plt.scatter(stat_x, stat_y,
            c='red', s=50, alpha=0.7,
            label=f'Stationary GPS (n={len(stat_x)})')
plt.scatter(0, 0, c='green', s=300, marker='*',
            label='True position (mean)')

# Draw 1-sigma circle
radius = np.mean([std_x, std_y])
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(radius*np.cos(theta), radius*np.sin(theta),
         'b--', linewidth=2,
         label=f'1-sigma radius = {radius:.3f}m')

plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title(f'Stationary GPS Noise Analysis\n'
          f'Std X = {std_x:.4f}m,  Std Y = {std_y:.4f}m\n'
          f'R matrix = diag([{var_x:.4f}, {var_y:.4f}])')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('/home/rian/ros2_ws/src/ekf_gps_imu/analysis/plot5_stationary.png', dpi=150)
plt.show()
print("Plot 5 saved!")

print("\nAll plots saved!")
print(f"\nReal R matrix from stationary data:")
print(f"self.R = np.diag([{var_x:.4f}, {var_y:.4f}])")