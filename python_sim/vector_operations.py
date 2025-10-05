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


def normalize_angle(theta):
    """Normalize angle to [-π, π)."""
    while theta >= np.pi:
        theta -= 2 * np.pi
    while theta < -np.pi:
        theta += 2 * np.pi
    return theta


def normalize_angle_0_to_2pi(angle):
    """Приводит угол к диапазону [0, 2π)"""
    return angle % (2 * np.pi)


def normalize_angle_minus_pi_to_pi(angle):
    """Приводит угол к диапазону [-π, π)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def find_intersection_vector_ellipse(vector, ellipse, eps=1e-4):
    # x1, y1 - robot pose; x2, y2 - lidar ray end
    x1, x2, y1, y2 = vector[0,0], vector[1,0], vector[0,1], vector[1,1]
    xc, yc, a, b = ellipse

    if abs(x2 - x1) < eps:
        A = 1
        B = -yc
        C = yc**2 - (1 - (x1 - xc)**2 / a**2) * b**2

        D = B**2 - A*C

        if D < 0:
            return None
        elif D < 1e-4:
            p_y = -B / A
        else:
            p1_y = (-B + np.sqrt(D)) / A
            d1 = (x1 - x1)**2 + (p1_y - y1)**2

            p2_y = (-B - np.sqrt(D)) / A
            d2 = (x1 - x1)**2 + (p2_y - y1)**2
            
            if d1 < d2:
                p_y = p1_y
            else:
                p_y = p2_y
    else:
        k = (y2 - y1) / (x2 - x1)
        S = y1 - k * x1 - yc
        V = a**2 * b**2

        A = b**2 + a**2 * k**2
        B = a**2 * k * S - b**2 * xc
        C = b**2 * xc**2 + a**2 * S**2 - V

        D = B**2 - A*C

        if D < 0:
            return None
        elif D < 1e-4:
            p_x = -B / A
            p_y = k * p_x + (y1 - k*x1)
        else:
            p1_x = (-B + np.sqrt(D)) / A
            p1_y = k * p1_x + (y1 - k*x1)
            d1 = (p1_x - x1)**2 + (p1_y - y1)**2

            p2_x = (-B - np.sqrt(D)) / A
            p2_y = k * p2_x + (y1 - k*x1)
            d2 = (p2_x - x1)**2 + (p2_y - y1)**2

            if d1 < d2:
                p_x = p1_x 
                p_y = p1_y
            else:
                p_x = p2_x 
                p_y = p2_y
    
    dot = (p_x - x1)*(x2 - x1) + (p_y - y1)*(y2 - y1)
    if dot < 0:
        return None

    squared_length_ab = (x2 - x1)**2 + (y2 - y1)**2
    if dot > squared_length_ab:
        return None
    
    return np.array([p_x, p_y])


def rotated_ellipse_coefficients(xc, yc, a, b, theta):
    A = (np.cos(theta)**2 / a**2) + (np.sin(theta)**2 / b**2)
    B = 2 * np.cos(theta) * np.sin(theta) * (1/a**2 - 1/b**2)
    C = (np.sin(theta)**2 / a**2) + (np.cos(theta)**2 / b**2)
    
    D = -2 * xc * A - 2 * yc * np.cos(theta) * np.sin(theta) * (1/a**2 - 1/b**2)
    E = -2 * yc * C - 2 * xc * np.cos(theta) * np.sin(theta) * (1/a**2 - 1/b**2)
    
    F = (xc**2 * A) + (yc**2 * C) + 2 * xc * yc * np.cos(theta) * np.sin(theta) * (1/a**2 - 1/b**2) - 1
    
    return A, B, C, D, E, F


def find_intersection_vector_ellipse_rot(vector, ellipse, eps=1e-4):
    # x1, y1 - robot pose; x2, y2 - lidar ray end
    # ellipse: [xc, yc, a, b, theta] - list
    x1, x2, y1, y2 = vector[0,0], vector[1,0], vector[0,1], vector[1,1]

    A, B, C, D_val, E, F = rotated_ellipse_coefficients(*ellipse)

    if abs(x2 - x1) < eps:
        p_x = x2
        a_ = C
        b_ = B * x2 + E
        c_ = A * x2**2 + B * x2 + F
        D = b_**2 - 4 * a_ * c_
        if D < 0:
            return None
        elif D < eps:
            p_y = - b_ / (2 * a_)
        else:
            p1_y = (-b_ + np.sqrt(D)) / (2 * a_)
            d1 = (p1_y - y1)**2

            p2_y = (-b_ - np.sqrt(D)) / (2 * a_)
            d2 = (p2_y - y1)**2
            
            if d1 < d2:
                p_y = p1_y
            else:
                p_y = p2_y
    else:
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k*x1

        a_ = A + B * k + C * k**2
        b_ = B * b + 2 * C * k * b + D_val + E * k
        c_ = C * b**2 + E * b + F
        D = b_**2 - 4 * a_ * c_
        if D < 0:
            return None
        elif D < eps:
            p_x = - b_ / (2 * a_)
            p_y = k * p_x + b
        else:
            p1_x = (-b_ + np.sqrt(D)) / (2 * a_)
            p1_y = k * p1_x + b
            d1 = (p1_x - x1)**2 + (p1_y - y1)**2

            p2_x = (-b_ - np.sqrt(D)) / (2 * a_)
            p2_y = k * p2_x + b
            d2 = (p2_x - x1)**2 + (p2_y - y1)**2

            if d1 < d2:
                p_x = p1_x 
                p_y = p1_y
            else:
                p_x = p2_x 
                p_y = p2_y
    
    dot = (p_x - x1)*(x2 - x1) + (p_y - y1)*(y2 - y1)
    if dot < 0:
        return None

    squared_length_ab = (x2 - x1)**2 + (y2 - y1)**2
    if dot > squared_length_ab:
        return None
    
    return np.array([p_x, p_y])