import numpy as np

def calculate_angular_velocity(V, bank_angle_deg):
    """
    This function calculates the angular velocity of an aircraft in both the inertial and body frames.
    """
    # Convert bank angle to radians
    bank_angle_rad = np.radians(bank_angle_deg)
    
    # Calculate turn radius
    g = 9.81
    R = V**2 / (g * np.tan(bank_angle_rad))
    
    # Calculate angular velocity
    omega = V / R  # Equivalent: omega = (g * np.tan(bank_angle_rad)) / V
    
    # Angular velocity vector in the inertial frame (only Yaw component)
    omega_inertial = [0, 0, omega]
    
    # Angular velocity vector in the body frame
    omega_body = [0, omega * np.sin(bank_angle_rad), omega * np.cos(bank_angle_rad)]

    return {
        'Angular_velocity_in_inertial_frame': omega_inertial,
        'Angular_velocity_in_body_frame': omega_body
    }

# Example usage
result = calculate_angular_velocity(V=250, bank_angle_deg=60)
print("Angular velocity calculations:")
print(result)

import numpy as np
import math

def compute_euler_angles(omega):
    """
    Computes the Euler angles and their rates from the given angular velocity vector.
    
    Parameters:
    omega : list
        A list containing the angular velocity components [P, Q, R].
    
    Returns:
    dict
        A dictionary with keys 'Euler_angles' and 'Euler_angles_rate', each containing
        a list of the computed values.
    """
    P, Q, R = omega
    
    # Initial Euler angles (assuming starting at zero for simplicity)
    phi = 0.0  # Roll
    theta = 0.0  # Pitch
    psi = 0.0  # Yaw
    
    # Compute Euler angle rates
    phi_dot = P + Q * np.sin(phi) * np.tan(theta) + R * np.cos(phi) * np.tan(theta)
    theta_dot = Q * np.cos(phi) - R * np.sin(phi)
    psi_dot = (Q * np.sin(phi) + R * np.cos(phi)) / np.cos(theta)
    
    # Store Euler angles and their rates
    euler_angles = [np.degrees(phi), np.degrees(theta), np.degrees(psi)]
    euler_angles_rate = [np.degrees(phi_dot), np.degrees(theta_dot), np.degrees(psi_dot)]
    
    return {
        "Euler_angles": euler_angles,
        "Euler_angles_rate": euler_angles_rate
    }

# Example usage
omega = [0.33, 0.28, 0.16]  # Angular velocity vector [P, Q, R]

# Compute and print the Euler angles and their rates
result = compute_euler_angles(omega)
print("\nComputed Euler Angles and Their Rates:")
print(result)


import numpy as np

def process_transformation_matrix(C):
    """
    Validates whether the given matrix is a rotation matrix and computes Euler angles,
    quaternion, and rotation vector if valid.
    
    Parameters:
    C : np.array
        A 3x3 transformation matrix.
    
    Returns:
    dict
        A dictionary containing either an error message or computed transformation parameters.
    """
    # Identity matrix for checking orthogonality
    I = np.eye(3)
    is_orthogonal = np.allclose(np.dot(C.T, C), I)
    det_C = np.linalg.det(C)
    is_det_one = np.isclose(det_C, 1.0)
    
    if not (is_orthogonal and is_det_one):
        return {
            "Error": "Matrix does not satisfy rotational matrix conditions.",
            "Orthogonality": is_orthogonal,
            "Determinant": det_C
        }
    
    # Compute Euler angles
    theta = np.arcsin(-C[2, 0])
    psi = np.arctan2(C[1, 0] / np.cos(theta), C[0, 0] / np.cos(theta))
    phi = np.arctan2(C[2, 1] / np.cos(theta), C[2, 2] / np.cos(theta))
    euler_angles = [np.degrees(psi), np.degrees(theta), np.degrees(phi)]
    
    # Compute Quaternion
    q0 = 0.5 * np.sqrt(1 + C[0, 0] + C[1, 1] + C[2, 2])
    q1 = (C[1, 2] - C[2, 1]) / (4 * q0)
    q2 = (C[2, 0] - C[0, 2]) / (4 * q0)
    q3 = (C[0, 1] - C[1, 0]) / (4 * q0)
    quaternion = [q0, q1, q2, q3]
    
    # Compute Rotation Vector
    rotation_vector = [psi, theta, phi]
    
    return {
        "Euler_angles": euler_angles,
        "Quaternion_vector": quaternion,
        "Rotation_vector": rotation_vector
    }

# Given matrix from the problem
C = np.array([
    [0.2802, 0.1387, 0.9499],
    [0.1962, 0.9603, -0.1981],
    [-0.9397, 0.2418, 0.2418]
])

# Process transformation matrix
result_transformation_matrix = process_transformation_matrix(C)
print("\nTransformation Matrix Parameters:")
print(result_transformation_matrix)


import numpy as np
import math

def compute_rotation_matrix(omega, flight_phase):
    """
    Computes the rotation matrix f(ω) for a given flight phase.
    
    Parameters:
    omega : list
        A list containing the angular velocity components [P, Q, R].
    flight_phase : str
        A string indicating the flight phase ('Cruise', 'Pull-up', or 'Coordinated-turn').
    
    Returns:
    np.array
        A 3x3 rotation matrix corresponding to the given flight phase.
    """
    omega_x, omega_y, omega_z = omega
    Omega = np.array([
        [0, -omega_z, omega_y],
        [omega_z, 0, -omega_x],
        [-omega_y, omega_x, 0]
    ])
    
    if flight_phase == "Cruise":
        return Omega  # Cruise mode transformation
    elif flight_phase == "Pull-up":
        return Omega  # Pull-up mode transformation
    elif flight_phase == "Coordinated-turn":
        return Omega  # Coordinated-turn transformation
    else:
        return "Invalid flight phase. Choose from 'Cruise', 'Pull-up', or 'Coordinated-turn'."

# Example usage
omega = [0.33, 0.28, 0.16]  # Angular velocity vector [P, Q, R]
flight_phase = "Cruise"  # Change to "Pull-up" or "Coordinated-turn" for other cases

# Compute and print the rotation matrix
rotation_matrix = compute_rotation_matrix(omega, flight_phase)
print("\nRotation Matrix for Flight Phase:")
print(rotation_matrix)
