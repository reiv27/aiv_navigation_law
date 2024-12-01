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

    # Normalize AB and AC
    VP1_norm = VP1 / np.linalg.norm(VP1)
    VP2_norm = VP2 / np.linalg.norm(VP2)

    # Check dot product signs
    dot1 = np.dot(VP1_norm, VR)
    dot2 = np.dot(VP2_norm, VR)

    # Check angle consistency
    cross = np.cross(VP1_norm, VP2_norm)
    return dot1 >= 0 and dot2 >= 0 and (np.cross(VP1, VR) * cross >= 0)


def find_vector_with_dir(point_1, point_2, l):
    dir = point_1 - point_2
    unit_dir = dir / np.linalg.norm(dir)
    return point_2 + unit_dir * l