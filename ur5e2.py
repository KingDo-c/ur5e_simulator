import numpy as np
import open3d as o3d
import trimesh
from scipy.optimize import minimize

# Define joint parameters (symbolic representation replaced with numerical values)
d0 = 0.089159  # meter
a1 = 0.425
a2 = 0.39225
d3 = 0.10915
d4 = 0.09465
d5 = 0.0823

def Mq(theta, axis):
    if axis == 'x':
        return np.array([[1, 0, 0, 0],
                         [0, np.cos(theta), -np.sin(theta), 0],
                         [0, np.sin(theta), np.cos(theta), 0],
                         [0, 0, 0, 1]])
    elif axis == 'y':
        return np.array([[np.cos(theta), 0, np.sin(theta), 0],
                         [0, 1, 0, 0],
                         [-np.sin(theta), 0, np.cos(theta), 0],
                         [0, 0, 0, 1]])
    elif axis == 'z':
        return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                         [np.sin(theta), np.cos(theta), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")

# Forward Kinematics function
def forward_kinematics(q_values):
    # Pad q_values with zeros if less than 6 values are provided
    q_values = list(q_values) + [0] * (6 - len(q_values))
    q1, q2, q3, q4, q5, q6 = q_values

    M_01 = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d0],
                     [0, 0, 0, 1]]) @ Mq(q1, 'z')

    M_12 = np.array([[0, 0, 1, 0],
                     [1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]]) @ Mq(q2, 'y')

    M_23 = np.array([[1, 0, 0, 0],
                     [0, 1, 0, a1],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]) @ Mq(q3, 'y')

    M_34 = np.array([[1, 0, 0, 0],
                     [0, 1, 0, a2],
                     [0, 0, 1, d3],
                     [0, 0, 0, 1]]) @ Mq(q4, 'y')

    M_45 = np.array([[0, 1, 0, 0],
                     [0, 0, 1, d4],
                     [1, 0, 0, 0],
                     [0, 0, 0, 1]]) @ Mq(q5, 'x')

    M_56 = np.array([[0, 0, 1, d5],
                     [1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]]) @ Mq(q6, 'y')

    M_06 = M_01 @ M_12 @ M_23 @ M_34 @ M_45 @ M_56
    return M_06

# Jacobian calculation function
def compute_jacobian(q_values):
    # Use forward kinematics to get transformation matrices
    M_01 = forward_kinematics(q_values[:1])
    M_02 = forward_kinematics(q_values[:2])
    M_03 = forward_kinematics(q_values[:3])
    M_04 = forward_kinematics(q_values[:4])
    M_05 = forward_kinematics(q_values[:5])
    M_06 = forward_kinematics(q_values)

    p_06 = M_06[:3, 3]

    # Calculate z axes for each joint (rotation axis)
    z_axes = [
        np.array([0, 0, 1]),
        (M_01[:3, :3] @ np.array([0, 1, 0])),
        M_02[:3, :3] @ np.array([0, 1, 0]),
        M_03[:3, :3] @ np.array([0, 1, 0]),
        M_04[:3, :3] @ np.array([1, 0, 0]),
        M_05[:3, :3] @ np.array([0, 1, 0])
    ]

    # Calculate positions for each joint
    p_origins = [np.array([0, 0, 0]), M_01[:3, 3], M_02[:3, 3], M_03[:3, 3], M_04[:3, 3], M_05[:3, 3]]

    # Calculate Jacobian columns
    J = np.zeros((6, 6))
    for i in range(6):
        z = z_axes[i]
        p = p_origins[i]
        J_p = np.cross(z, (p_06 - p))
        J_w = z
        J[:3, i] = J_p
        J[3:, i] = J_w

    return J

# Inverse Kinematics function using numerical optimization
def inverse_kinematics(target_position, initial_guess):
    def objective_function(q):
        M_06 = forward_kinematics(q)
        p_06 = M_06[:3, 3]
        return np.linalg.norm(p_06 - target_position)

    result = minimize(objective_function, initial_guess, method='BFGS')
    return result.x if result.success else None

# Test the functions
q_values = [-6.27278729e-01, -9.92879098e-01,  2.68779239e-06, -7.05446886e-01, -1.36764436e+00,  0.00000000e+00]
M_06 = forward_kinematics(q_values)
print("Forward Kinematics (End-Effector Pose):\n", M_06)

jacobian = compute_jacobian(q_values)
print("Jacobian Matrix:\n", jacobian)

target_position = np.array([1, 1, 1])
initial_guess = [0, 0, 0, 0, 0, 0]
q_solution = inverse_kinematics(target_position, initial_guess)
print("Inverse Kinematics Solution (Joint Angles):\n", q_solution)
