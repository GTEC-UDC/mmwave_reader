import numpy as np
import math

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def apply_rotation_matrix(point, rotation_matrix):
    # Create a numpy array from the point coordinates
    point_array = np.array([[point.x], [point.y], [point.z]])
    
    # Apply the rotation matrix to the point
    rotated_point = rotation_matrix.dot(point_array)
    
    # Extract the rotated coordinates from the numpy array
    rotated_x, rotated_y, rotated_z = rotated_point.flatten()
    
    # Create a new Point object with the rotated coordinates
    rotated_point_obj = Point(rotated_x, rotated_y, rotated_z)
    
    return rotated_point_obj

def euler_to_rotation_matrix(yaw, pitch, roll):

    # Calculate the trigonometric values
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    # Calculate the elements of the rotation matrix
    rotation_matrix = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                                [-sp, cp * sr, cp * cr]])
    
    return rotation_matrix

# # Example usage
# # Define the Euler angles
# yaw = 45  # Yaw angle in degrees
# pitch = 30  # Pitch angle in degrees
# roll = 60  # Roll angle in degrees

# # Convert Euler angles to rotation matrix
# rotation_matrix = euler_to_rotation_matrix(yaw, pitch, roll)

# # Create a point object
# point = Point(1, 2, 3)

# # Apply the rotation matrix to the point
# rotated_point = apply_rotation_matrix(point, rotation_matrix)

# # Print the rotated coordinates
# print("Rotated point coordinates:")
# print(f"x: {rotated_point.x}")
# print(f"y: {rotated_point.y}")
# print(f"z: {rotated_point.z}")
