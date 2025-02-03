import numpy as np
import matplotlib.pyplot as plt

# LiDAR scan data (example)
ranges = [float('inf')] * 100 + [2.7861766815185547, 2.755803108215332, 2.7421741485595703, 2.7338345050811768, 2.7061517238616943, 
                                 2.6977744102478027, 2.6607651710510254, 2.6619160175323486, 2.645494222640991, 2.633470058441162, 
                                 2.6228115558624268, 2.6202468872070312, 2.574528932571411, 2.596367835998535, 2.5783722400665283, 
                                 2.569840908050537, 2.5737926959991455, 2.803567409515381, 3.3184733390808105] + [float('inf')] * 100
intensities = [0.0] * len(ranges)  # Intensities array as given (not used for the plot)

# Generate angles (assuming 360 degrees scan with an angular increment of ~1 degree)
angle_min = 0.0
angle_max = 2 * np.pi  # 360 degrees in radians
angle_increment = 2 * np.pi / len(ranges)  # Calculate the increment based on the range length
angles = np.arange(angle_min, angle_max, angle_increment)

# Filter out "inf" values as they indicate no detection at that angle
valid_ranges = [r if r != float('inf') else np.nan for r in ranges]

# Create polar plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')

# Plot the data
ax.plot(angles, valid_ranges, marker='o', linestyle='-', color='b')

# Set labels and title
ax.set_title("LiDAR Scan - Distance vs Angle")
ax.set_xlabel("Angle (radians)")
ax.set_ylabel("Distance (meters)")

# Display the plot
plt.show()
