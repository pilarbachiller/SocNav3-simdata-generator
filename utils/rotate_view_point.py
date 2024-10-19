import numpy as np

# Function to convert axis-angle to quaternion
def axis_angle_to_quaternion(axis, angle):
    half_angle = angle / 2
    sin_half_angle = np.sin(half_angle)
    return np.array([axis[0] * sin_half_angle, axis[1] * sin_half_angle, axis[2] * sin_half_angle, np.cos(half_angle)])

# Function to multiply two quaternions
def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
        w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ])

# Function to convert quaternion back to axis-angle
def quaternion_to_axis_angle(quaternion):
    q = quaternion / np.linalg.norm(quaternion)
    angle = 2 * np.arccos(q[3])
    s = np.sqrt(1 - q[3]**2)
    if s < 0.001:  # To avoid division by zero
        x = q[0]
        y = q[1]
        z = q[2]
    else:
        x = q[0] / s
        y = q[1] / s
        z = q[2] / s
    return np.array([x, y, z]), angle

# Existing top view orientation (axis-angle)
existing_axis = np.array([-0.57, 0.57, 0.57])
existing_angle = 2.09

# Convert to quaternion
existing_quaternion = axis_angle_to_quaternion(existing_axis, existing_angle)

# Z-axis rotation (45 degrees)
z_axis = np.array([1, 0, 0])
z_angle = -2.10 - np.pi / 2. 

# Convert to quaternion
z_rotation_quaternion = axis_angle_to_quaternion(z_axis, z_angle)

# Calculate the new orientation quaternion
new_orientation_quaternion = quaternion_multiply(existing_quaternion, z_rotation_quaternion)

# Convert quaternion back to axis-angle
new_axis, new_angle = quaternion_to_axis_angle(new_orientation_quaternion)

# Print the new axis-angle representation
print("New axis:", new_axis)
print("New angle (radians):", new_angle)
print("New angle (degrees):", np.degrees(new_angle))
