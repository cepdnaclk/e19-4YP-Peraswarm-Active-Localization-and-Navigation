import numpy as np
import matplotlib.pyplot as plt

# LiDAR simulation parameters
num_points = 360  # 1-degree resolution
angle_min = 0.0
angle_max = 2 * np.pi
angle_increment = (angle_max - angle_min) / num_points
angles = np.linspace(angle_min, angle_max, num_points, endpoint=False)

# Simulated ranges (in meters)
ranges = np.full(num_points, float('inf'))

# Simulate walls and objects
# Add a circular wall at 3 meters (from 45° to 135°)
for i in range(45, 135):
    ranges[i] = 3.0 + np.random.normal(0, 0.05)  # small noise

# Add a box-shaped obstacle from 210° to 240° at ~2m
for i in range(210, 240):
    ranges[i] = 2.0 + np.random.normal(0, 0.05)

# Add a pole at 315° at 1.2m
ranges[315] = 1.2

# Convert polar to Cartesian for plotting
x_points = [r * np.cos(angle) if np.isfinite(r) else np.nan for r, angle in zip(ranges, angles)]
y_points = [r * np.sin(angle) if np.isfinite(r) else np.nan for r, angle in zip(ranges, angles)]

# Plot simulated LiDAR points
plt.figure(figsize=(8, 8))
plt.plot(x_points, y_points, 'bo', markersize=2)
plt.title("Simulated 2D LiDAR Sensor Data")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.axis("equal")
plt.show()

# Optional: print a few sample values
print("Sample LiDAR ranges (degrees 0–10):")
for i in range(10):
    print(f"Angle {i}°: {ranges[i]:.2f} m")
