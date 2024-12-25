import numpy as np


def find_intersection(vector_1, vector_2):
    # Compute direction vectors
    d1 = vector_1[1] - vector_1[0]
    d2 = vector_2[1] - vector_2[0]

    # Set up the linear system for intersection
    A = np.array([d1, -d2]).T
    b = vector_2[0] - vector_1[0]

    # Solve for intersection if not parallel
    if np.linalg.matrix_rank(A) == 2:
        t, s = np.linalg.solve(A, b)
        if 0 <= t <= 1 and 0 <= s <= 1:
            return vector_1[0] + t * d1


def is_point_in_angle(V, P1, P2, r_pose):
    # Calculate vectors
    VP1 = P1 - V
    VP2 = P2 - V
    VR = r_pose - V

    # Normalize vectors
    VP1_norm = VP1 / np.linalg.norm(VP1)
    VP2_norm = VP2 / np.linalg.norm(VP2)
    VR_norm = VR / np.linalg.norm(VR)

    # Calculate the angle between P1 and P2 using the dot product
    angle_P1_P2 = np.arccos(np.clip(np.dot(VP1_norm, VP2_norm), -1.0, 1.0))  # Angle between P1 and P2

    # Calculate the angle between VR and VP1
    angle_VR_P1 = np.arccos(np.clip(np.dot(VR_norm, VP1_norm), -1.0, 1.0))

    # Calculate the angle between VR and VP2
    angle_VR_P2 = np.arccos(np.clip(np.dot(VR_norm, VP2_norm), -1.0, 1.0))

    # Check if the angle between VR and both P1 and P2 is less than the angle between P1 and P2
    return angle_VR_P1 <= angle_P1_P2 and angle_VR_P2 <= angle_P1_P2


def find_vector_with_dir(point_1, point_2, l):
    dir = point_1 - point_2
    unit_dir = dir / np.linalg.norm(dir)
    return point_2 + unit_dir * l


def saturation(x, min_val, max_val):
    return np.clip(x, min_val, max_val)