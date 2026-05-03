import numpy as np
from scipy.spatial.transform import Rotation as R

# Existing quaternion (x, y, z, w)
q_current = [0.06517687, 0.60561229, -0.08403885, 0.78862107]

# Quaternion for 90-degree anticlockwise rotation around Z-axis
q_z = [0, 0, 0.7071, 0.7071]

# Convert to scipy Rotation objects
rotation_current = R.from_quat(q_current)
rotation_z = R.from_quat(q_z)

# Combine the rotations
rotation_new = rotation_z * rotation_current

# Get the resulting quaternion
q_new = rotation_new.as_quat()  # [x, y, z, w]

# Translation remains unchanged
translation = [0.2607, 0.1627, 0.6679]

# Print the results
print("Translation (ROS2):", translation)
print("Quaternion (ROS2):", q_new)
