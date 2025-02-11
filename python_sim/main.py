import json
import numpy as np
import time as TIME
import matplotlib.pyplot as plt

from robot import *
from vector_operations import find_vector_with_dir

def calculate_arc_points(start, end, center, radius, num_points=100):
    start = np.array(start, dtype=np.float64)
    end = np.array(end, dtype=np.float64)
    center = np.array(center, dtype=np.float64)

    theta_start = np.arctan2(start[1] - center[1], start[0] - center[0])
    theta_end = np.arctan2(end[1] - center[1], end[0] - center[0])

    if theta_start < theta_end:
        theta_start += 2 * np.pi

    theta = np.linspace(theta_start, theta_end, num_points)

    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)

    return np.column_stack((x, y))


def parallel_line(points, distance):
    points = np.array(points, dtype=np.float64)
    p1, p2 = points

    direction = p2 - p1

    norm_direction = direction / np.linalg.norm(direction)

    normal = np.array([-norm_direction[1], norm_direction[0]])

    offset = distance * normal
    parallel_p1 = p1 + offset
    parallel_p2 = p2 + offset

    return np.array([parallel_p1, parallel_p2])





# # Obstacles and equidistant 1
# obs_1 = np.array([[50, 10], [50, 65]], dtype=np.float32)
# obs_2 = np.array([[50, 65], [0, 65]], dtype=np.float32)
# obs_3 = np.array([[0, 65],  [0, 10]], dtype=np.float32)
# obs_4 = np.array([[0, 10],   [50, 10]], dtype=np.float32)

# equid_1 = np.array([[42, 18], [42, 57]], dtype=np.float32)
# equid_2 = np.array([[42, 57], [8, 57]], dtype=np.float32)
# equid_3 = np.array([[8, 57], [8, 18]], dtype=np.float32)
# equid_4 = np.array([[8, 18], [42, 18]], dtype=np.float32)

# obstacles = np.array([obs_1, obs_2, obs_3, obs_4])
# equidistants = np.array([equid_1, equid_2, equid_3, equid_4])


# obs_1 = np.array([[0, 65],  [0, 10]], dtype=np.float32)
# obs_2 = np.array([[0, 10],   [50, 10]], dtype=np.float32)
# obs_3 = np.array([[50, 10], [50, 30]], dtype=np.float32)
# obs_4 = np.array([[50, 30], [10, 30]], dtype=np.float32)
# obs_5 = np.array([[10, 30], [10, 40]], dtype=np.float32)
# obs_6 = np.array([[10, 40], [50, 40]], dtype=np.float32)
# obs_7 = np.array([[50, 40], [50, 65]], dtype=np.float32)
# obs_8 = np.array([[50, 65], [0, 65]], dtype=np.float32)
# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8])


obs_1 = np.array([[0, 60],  [0, 10]], dtype=np.float32)
obs_2 = np.array([[0, 10],   [50, 10]], dtype=np.float32)
obs_3 = np.array([[50, 10], [10, 35]], dtype=np.float32)
obs_4 = np.array([[10, 35], [50, 60]], dtype=np.float32)
# obs_4 = np.array([[50, 30], [10, 30]], dtype=np.float32)
# obs_5 = np.array([[10, 30], [10, 40]], dtype=np.float32)
# obs_6 = np.array([[10, 40], [50, 40]], dtype=np.float32)
# obs_7 = np.array([[50, 40], [50, 60]], dtype=np.float32)
obs_5 = np.array([[50, 60], [0, 60]], dtype=np.float32)
obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5])



# Obstacles 2
# obs_1 = np.array([[0, 0], [70, 0]], dtype=np.float64)
# obs_2 = np.array([[70, 0], [70, 50]], dtype=np.float64)
# obs_3 = np.array([[70, 50], [150, 50]], dtype=np.float64)
# obs_4 = np.array([[150, 50], [150, 0]], dtype=np.float64)
# obs_5 = np.array([[150, 0], [220, 0]], dtype=np.float64)
# obs_6 = np.array([[220, 0], [220, 140]], dtype=np.float64)
# obs_7 = np.array([[220, 140], [150, 140]], dtype=np.float64)
# obs_8 = np.array([[150, 140], [150, 90]], dtype=np.float64)
# obs_9 = np.array([[150, 90], [70, 90]], dtype=np.float64)
# obs_10 = np.array([[70, 90], [70, 140]], dtype=np.float64)
# obs_11 = np.array([[70, 140], [0, 140]], dtype=np.float64)
# obs_12 = np.array([[0, 140], [0, 0]], dtype=np.float64)
# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8, obs_9, obs_10, obs_11, obs_12])

# equid_1 = np.array([[8, 8], [62, 8]], dtype=np.float64)
# equid_2 = np.array([[62, 8], [62, 50]], dtype=np.float64)
# arc_1 = calculate_arc_points([62, 50], [70, 58], [70, 50], 8)
# equid_3 = np.array([[70, 58], [150, 58]], dtype=np.float64)
# arc_2 = calculate_arc_points([150, 58], [158, 50], [150, 50], 8)
# equid_4 = np.array([[158, 50], [158, 8]], dtype=np.float64)
# equid_5 = np.array([[158, 8], [212, 8]], dtype=np.float64)
# equid_6 = np.array([[212, 8], [212, 132]], dtype=np.float64)
# equid_7 = np.array([[212, 132], [158, 132]], dtype=np.float64)
# equid_8 = np.array([[158, 132], [158, 90]], dtype=np.float64)
# arc_3 = calculate_arc_points([158, 90], [150, 82], [150, 90], 8)
# equid_9 = np.array([[150, 82], [70, 82]], dtype=np.float64)
# arc_4 = calculate_arc_points([70, 82], [62, 90], [70, 90], 8)
# equid_10 = np.array([[62, 90], [62, 132]], dtype=np.float64)
# equid_11 = np.array([[62, 132], [8, 132]], dtype=np.float64)
# equid_12 = np.array([[8, 132], [8, 8]], dtype=np.float64)

# arc_1 = calculate_arc_points([0, -8], [-8, 0], [0, 0], 8)
# equid_1 = np.array([[0, -8], [70, -8]], dtype=np.float64)
# arc_2 = calculate_arc_points([78, 0], [70, -8], [70, 0], 8)
# equid_2 = np.array([[78, 0], [78, 42]], dtype=np.float64)
# equid_3 = np.array([[78, 42], [142, 42]], dtype=np.float64)
# equid_4 = np.array([[142, 42], [142, 0]], dtype=np.float64)
# arc_3 = calculate_arc_points([150, -8], [142, 0], [150, 0], 8)
# equid_5 = np.array([[150, -8], [220, -8]], dtype=np.float64)
# arc_4 = calculate_arc_points([228, 0], [220, -8], [220, 0], 8)
# equid_6 = np.array([[228, 0], [228, 140]], dtype=np.float64)
# arc_5 = calculate_arc_points([220, 148], [228, 140], [220, 140], 8)
# equid_7 = np.array([[220, 148], [150, 148]], dtype=np.float64)
# arc_6 = calculate_arc_points([142, 140], [150, 148], [150, 140], 8)
# equid_8 = np.array([[142, 140], [142, 98]], dtype=np.float64)
# equid_9 = np.array([[142, 98], [78, 98]], dtype=np.float64)
# equid_10 = np.array([[78, 98], [78, 140]], dtype=np.float64)
# arc_7 = calculate_arc_points([70, 148], [78, 140], [70, 140], 8)
# equid_11 = np.array([[70, 148], [0, 148]], dtype=np.float64)
# arc_8 = calculate_arc_points([-8, 140], [0, 148], [0, 140], 8)
# equid_12 = np.array([[-8, 140], [-8, 0]], dtype=np.float64)

# equidistants = np.array([equid_1, equid_2, equid_3, equid_4, equid_5, equid_6, equid_7, equid_8, equid_9, equid_10, equid_11, equid_12])





# Obstacles 3
# obs_1 = np.array([[0, 0], [80, 55]], dtype=np.float64)
# obs_2 = np.array([[80, 55], [140, 55]], dtype=np.float64)
# obs_3 = np.array([[140, 55], [220, 0]], dtype=np.float64)
# obs_4 = np.array([[220, 0], [220, 140]], dtype=np.float64)
# obs_5 = np.array([[220, 140], [140, 85]], dtype=np.float64)
# obs_6 = np.array([[140, 85], [80, 85]], dtype=np.float64)
# obs_7 = np.array([[80, 85], [0, 140]], dtype=np.float64)
# obs_8 = np.array([[0, 140], [0, 0]], dtype=np.float64)
# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8])

# equid_1 = np.array([[8, 8], [8, 8]], dtype=np.float64)
# equid_2 = np.array([[62, 8], [62, 50]], dtype=np.float64)
# arc_1 = calculate_arc_points([62, 50], [70, 58], [70, 50], 8)
# equid_3 = np.array([[70, 58], [150, 58]], dtype=np.float64)
# arc_2 = calculate_arc_points([150, 58], [158, 50], [150, 50], 8)
# equid_4 = np.array([[158, 50], [158, 8]], dtype=np.float64)
# equid_5 = np.array([[158, 8], [212, 8]], dtype=np.float64)
# equid_6 = np.array([[212, 8], [212, 132]], dtype=np.float64)
# equid_7 = np.array([[212, 132], [158, 132]], dtype=np.float64)
# equid_8 = np.array([[158, 132], [158, 90]], dtype=np.float64)

# equid_1 = np.array([[8, 8], [62, 8]], dtype=np.float64)
# equid_2 = np.array([[62, 8], [62, 50]], dtype=np.float64)
# arc_1 = calculate_arc_points([62, 50], [70, 58], [70, 50], 8)
# equid_3 = np.array([[70, 58], [150, 58]], dtype=np.float64)
# arc_2 = calculate_arc_points([150, 58], [158, 50], [150, 50], 8)
# equid_4 = np.array([[158, 50], [158, 8]], dtype=np.float64)
# equid_5 = np.array([[158, 8], [212, 8]], dtype=np.float64)
# equid_6 = np.array([[212, 8], [212, 132]], dtype=np.float64)
# equid_7 = np.array([[212, 132], [158, 132]], dtype=np.float64)
# equid_8 = np.array([[158, 132], [158, 90]], dtype=np.float64)

# equidistants = np.array([equid_1, equid_2, equid_3, equid_4, equid_5, equid_6, equid_7, equid_8])





# Obstacles 4
# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obs_5 = np.array([[20, 4], [25, 4]], dtype=np.float64)
# obs_6 = np.array([[25, 4], [25, 9]], dtype=np.float64)
# obs_7 = np.array([[25, 9], [20, 9]], dtype=np.float64)
# obs_8 = np.array([[20, 9], [20, 4]], dtype=np.float64)

# obs_9 = np.array([[35, 2], [40, 2]], dtype=np.float64)
# obs_10 = np.array([[40, 2], [40, 7]], dtype=np.float64)
# obs_11 = np.array([[40, 7], [35, 7]], dtype=np.float64)
# obs_12 = np.array([[35, 7], [35, 2]], dtype=np.float64)

# obs_13 = np.array([[36, -7], [36, -12]], dtype=np.float64)
# obs_14 = np.array([[36, -12], [41, -12]], dtype=np.float64)
# obs_15 = np.array([[41, -12], [41, -7]], dtype=np.float64)
# obs_16 = np.array([[41, -7], [36, -7]], dtype=np.float64)

# obs_17 = np.array([[3, -10], [3, -15]], dtype=np.float64)
# obs_18 = np.array([[3, -15], [8, -15]], dtype=np.float64)
# obs_19 = np.array([[8, -15], [8, -10]], dtype=np.float64)
# obs_20 = np.array([[8, -10], [3, -10]], dtype=np.float64)

# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8,
#                       obs_9, obs_10, obs_11, obs_12, obs_13, obs_14, obs_15, obs_16,
#                       obs_17, obs_18, obs_19, obs_20])

# equid_1 = np.array([[0, -10], [5, -10]], dtype=np.float64)
# arc_1 = calculate_arc_points([15, 0], [5, -10], [5, 0], 10)
# equid_2 = np.array([[15, 0], [15, 5]], dtype=np.float64)
# arc_2 = calculate_arc_points([5, 15], [15, 5], [5, 5], 10)
# equid_3 = np.array([[5, 15], [0, 15]], dtype=np.float64)
# arc_3 = calculate_arc_points([-10, 5], [0, 15], [0, 5], 10)
# equid_4 = np.array([[-10, 5], [-10, 0]], dtype=np.float64)
# arc_4 = calculate_arc_points([0, -10], [-10, 0], [0, 0], 10)

# equid_5 = np.array([[20, -6], [25, -6]], dtype=np.float64)
# arc_5 = calculate_arc_points([35, 4], [25, -6], [25, 4], 10)
# equid_6 = np.array([[35, 4], [35, 9]], dtype=np.float64)
# arc_6 = calculate_arc_points([25, 19], [35, 9], [25, 9], 10)
# equid_7 = np.array([[25, 19], [20, 19]], dtype=np.float64)
# arc_7 = calculate_arc_points([10, 9], [20, 19], [20, 9], 10)
# equid_8 = np.array([[10, 9], [10, 4]], dtype=np.float64)
# arc_8 = calculate_arc_points([20, -6], [10, 4], [20, 4], 10)

# equid_9 = np.array([[35, -8], [40, -8]], dtype=np.float64)
# arc_9 = calculate_arc_points([50, 2], [40, -8], [40, 2], 10)
# equid_10 = np.array([[50, 2], [50, 7]], dtype=np.float64)
# arc_10 = calculate_arc_points([40, 17], [50, 7], [40, 7], 10)
# equid_11 = np.array([[40, 17], [35, 17]], dtype=np.float64)
# arc_11 = calculate_arc_points([25, 7], [35, 17], [35, 7], 10)
# equid_12 = np.array([[25, 7], [25, 2]], dtype=np.float64)
# arc_12 = calculate_arc_points([35, -8], [25, 2], [35, 2], 10)

# equid_13 = np.array([[26, -7], [26, -12]], dtype=np.float64)
# arc_13 = calculate_arc_points([36, -22], [26, -12], [36, -12], 10)
# equid_14 = np.array([[36, -22], [41, -22]], dtype=np.float64)
# arc_14 = calculate_arc_points([51, -12], [41, -22], [41, -12], 10)
# equid_15 = np.array([[51, -12], [51, -7]], dtype=np.float64)
# arc_15 = calculate_arc_points([41, 3], [51, -7], [41, -7], 10)
# equid_16 = np.array([[41, 3], [36, 3]], dtype=np.float64)
# arc_16 = calculate_arc_points([26, -7], [36, 3], [36, -7], 10)

# equid_17 = np.array([[-7, -10], [-7, -15]], dtype=np.float64)
# arc_17 = calculate_arc_points([3, -25], [-7, -15], [3, -15], 10)
# equid_18 = np.array([[3, -25], [8, -25]], dtype=np.float64)
# arc_18 = calculate_arc_points([18, -15], [8, -25], [8, -15], 10)
# equid_19 = np.array([[18, -15], [18, -10]], dtype=np.float64)
# arc_19 = calculate_arc_points([8, 0], [18, -10], [8, -10], 10)
# equid_20 = np.array([[8, 0], [3, 0]], dtype=np.float64)
# arc_20 = calculate_arc_points([-7, -10], [3, 0], [3, -10], 10)

# equidistants = np.array([equid_1, equid_2, equid_3, equid_4, equid_5, equid_6, equid_7, equid_8,
#                          equid_9, equid_10, equid_11, equid_12, equid_13, equid_14, equid_15, equid_16,
#                          equid_17, equid_18, equid_19, equid_20])






# Obstacles 6
# obs_1 = np.array([[0, 0], [0, 90]], dtype=np.float64)
# obs_2 = np.array([[0, 90], [30, 90]], dtype=np.float64)
# obs_3 = np.array([[30, 90], [30, 0]], dtype=np.float64)
# obs_4 = np.array([[30, 0], [0, 0]], dtype=np.float64)

# obs_5 = np.array([[30, 60], [30, 120]], dtype=np.float64)
# obs_6 = np.array([[30, 120], [170, 120]], dtype=np.float64)
# obs_7 = np.array([[170, 120], [170, 60]], dtype=np.float64)
# obs_8 = np.array([[170, 60], [30, 60]], dtype=np.float64)

# obs_9 = np.array([[30, -30], [30, 30]], dtype=np.float64)
# obs_10 = np.array([[30, 30], [100, 30]], dtype=np.float64)
# obs_11 = np.array([[100, 30], [100, -30]], dtype=np.float64)
# obs_12 = np.array([[100, -30], [30, -30]], dtype=np.float64)

# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8,
#                       obs_9, obs_10, obs_11, obs_12])

# equid_1 = np.array([[-8, 0], [-8, 90]], dtype=np.float64)
# arc_1 = calculate_arc_points([-8, 90], [0, 98], [0, 90], 8)
# equid_2 = np.array([[0, 98], [22, 98]], dtype=np.float64)
# equid_3 = np.array([[22, 98], [22, 120]], dtype=np.float64)
# arc_2 = calculate_arc_points([22, 120], [30, 128], [30, 120], 8)
# equid_4 = np.array([[30, 128], [170, 128]], dtype=np.float64)
# arc_3 = calculate_arc_points([170, 128], [178, 120], [170, 120], 8)
# equid_5 = np.array([[178, 120], [178, 60]], dtype=np.float64)
# arc_4 = calculate_arc_points([178, 60], [170, 52], [170, 60], 8)
# equid_6 = np.array([[170, 52], [38, 52]], dtype=np.float64)
# equid_7 = np.array([[38, 52], [38, 38]], dtype=np.float64)
# equid_8 = np.array([[38, 38], [100, 38]], dtype=np.float64)
# arc_5 = calculate_arc_points([100, 38], [108, 30], [100, 30], 8)
# equid_9 = np.array([[108, 30], [108, -30]], dtype=np.float64)
# arc_6 = calculate_arc_points([108, -30], [100, -38], [100, -30], 8)
# equid_10 = np.array([[100, -38], [30, -38]], dtype=np.float64)
# arc_7 = calculate_arc_points([30, -38], [22, -30], [30, -30], 8)
# equid_11 = np.array([[22, -30], [22, -8]], dtype=np.float64)
# equid_12 = np.array([[22, -8], [0, -8]], dtype=np.float64)
# arc_8 = calculate_arc_points([0, -8], [-8, 0], [0, 0], 8)

# equidistants = np.array([equid_1, equid_2, equid_3, equid_4, equid_5, equid_6, equid_7, equid_8, equid_9, equid_10, equid_11, equid_12])





# Obstacles 7
# obs_1 = np.array([[0, 0], [50, 0]], dtype=np.float64)
# obs_2 = np.array([[50, 0], [93.30, 25]], dtype=np.float64)
# equid_1 = np.array([[0, 10], [47.32, 10]], dtype=np.float64)
# equid_2 = np.array([[47.32, 10], [88.3, 33.66]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [50, 0]], dtype=np.float64)
# obs_2 = np.array([[50, 0], [75, 43.30]], dtype=np.float64)
# equid_1 = np.array([[0, 10], [44.22658, 10]], dtype=np.float64)
# equid_2 = np.array([[44.22658, 10], [66.3398, 48.3]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [50, 0]], dtype=np.float64)
# obs_2 = np.array([[50, 0], [50, 50]], dtype=np.float64)
# equid_1 = np.array([[0, 10], [40, 10]], dtype=np.float64)
# equid_2 = np.array([[40, 10], [40, 50]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [50, 0]], dtype=np.float64)
# obs_2 = np.array([[50, 0], [25, 43.30]], dtype=np.float64)
# equid_1 = np.array([[0, 10], [32.6792, 10]], dtype=np.float64)
# equid_2 = np.array([[32.6792, 10], [16.3398, 38.3]], dtype=np.float64)

# obs_1 = np.array([[-50, 0], [50, 0]], dtype=np.float64)
# obs_2 = np.array([[50, 0], [-40, 51.96]], dtype=np.float64)
# equid_1 = np.array([[-50, 10], [12.6785, 10]], dtype=np.float64)
# equid_2 = np.array([[12.6785, 10], [-45, 43.3]], dtype=np.float64)

# obstacles = np.array([obs_1, obs_2])
# equidistants = np.array([equid_1, equid_2])

# equid_1 = parallel_line(obs_1, 10)
# equid_2  = parallel_line(obs_2, 10)
# equidistants = np.array([equid_1, equid_2])


def plot_obstacles(obstacles):
    for obstacle in obstacles:
        plt.plot([obstacle[0][0], obstacle[1][0]], [obstacle[0][1], obstacle[1][1]], color='black')


def plot_equidistants(equidistants):
    for equidistant in equidistants:
        plt.plot([equidistant[0][0], equidistant[1][0]], [equidistant[0][1], equidistant[1][1]], color='blue', zorder=1)


start_time = TIME.time()


dt = 1                             # Time step
time_end = 3000                    # Simulation time
time = np.arange(0, time_end, dt)  # Total simulation time
d = 8                              # Distance from equididstant to objects

robot = DubinsCar(0.1, 0.02, 5, d)
robot.init_pose(25, 35, 0)

# Simulate
for t in time:
    # Update position based on Dubins car kinematics
    robot.update_pose(dt)
    # LiDAR scan
    robot.lidar_scan(obstacles)
    if t == 0:
        robot.goal = np.arctan2(robot.lidar.lidar_closest_point[1]-robot.y, robot.lidar.lidar_closest_point[0]-robot.x)
        # print(robot.lidar.lidar_closest_point)
        # print(robot.y, robot.x)
        # phi = np.arctan2(vy, vx)
    # Update disk rays
    robot.update_disk_center()
    robot.update_disk_rays()

    #Check global robot state
    # robot.switch_global_mode()

    # Check robot state
    robot.switch_mode_in_main()
    # Calculate U
    robot.calculate_u()
    # robot.calculate_global_u()
    print(robot.u)
    robot.update_orientation(dt)

    print(f'{t} / {time_end}')


stop_time = TIME.time()
print((stop_time-start_time) / 60)

def plotting_results():
    fig, ax = plt.subplots()
    # Plot the Dubins car path
    plt.plot(robot.x_path, robot.y_path, label="Dubins Car Path", color='red', zorder=2)
    plt.scatter(robot.x_path[0], robot.y_path[0], color='black', s=70, label="start point", zorder=3)
    plt.scatter(robot.x_path[-1], robot.y_path[-1], color='green', s=250, label="end point", marker='*', zorder=3)

    # Plot obstacles
    plot_obstacles(obstacles)
    # plot_equidistants(equidistants)
    
    # plt.plot(arc_1[:, 0], arc_1[:, 1], color='blue', zorder=1)
    # plt.plot(arc_2[:, 0], arc_2[:, 1], color='blue', zorder=1)
    # plt.plot(arc_3[:, 0], arc_3[:, 1], color='blue', zorder=1)
    # plt.plot(arc_4[:, 0], arc_4[:, 1], color='blue', zorder=1)

    # plt.plot(arc_5[:, 0], arc_5[:, 1], color='blue', zorder=1)
    # plt.plot(arc_6[:, 0], arc_6[:, 1], color='blue', zorder=1)
    # plt.plot(arc_7[:, 0], arc_7[:, 1], color='blue', zorder=1)
    # plt.plot(arc_8[:, 0], arc_8[:, 1], color='blue', zorder=1)

    # plt.plot(arc_9[:, 0], arc_9[:, 1], color='blue', zorder=1)
    # plt.plot(arc_10[:, 0], arc_10[:, 1], color='blue', zorder=1)
    # plt.plot(arc_11[:, 0], arc_11[:, 1], color='blue', zorder=1)
    # plt.plot(arc_12[:, 0], arc_12[:, 1], color='blue', zorder=1)
    # plt.plot(arc_13[:, 0], arc_13[:, 1], color='blue', zorder=1)
    # plt.plot(arc_14[:, 0], arc_14[:, 1], color='blue', zorder=1)
    # plt.plot(arc_15[:, 0], arc_15[:, 1], color='blue', zorder=1)
    # plt.plot(arc_16[:, 0], arc_16[:, 1], color='blue', zorder=1)
    # plt.plot(arc_17[:, 0], arc_17[:, 1], color='blue', zorder=1)
    # plt.plot(arc_18[:, 0], arc_18[:, 1], color='blue', zorder=1)
    # plt.plot(arc_19[:, 0], arc_19[:, 1], color='blue', zorder=1)
    # plt.plot(arc_20[:, 0], arc_20[:, 1], color='blue', zorder=1)



    # Plot LiDAR points
    # plt.scatter(robot.lidar.lidar_points[:,0], robot.lidar.lidar_points[:,1], color='red', s=1, label="LiDAR Rays")
    # print(robot.lidar.lidar_closest_point)
    
    # Plot closest point and circle center
    # disk = plt.Circle(robot.disk.disk_center, robot.R_min, color='orange', fill=False, linewidth=1)
    # ax.add_patch(disk)
    # plt.scatter(robot.lidar.lidar_closest_point[0], robot.lidar.lidar_closest_point[1], color='brown', s=50, label="Closest Robot Point")
    # plt.plot([robot.disk.disk_center[0], robot.lidar.lidar_points[robot.disk.min_arg][0]], [robot.disk.disk_center[1], robot.lidar.lidar_points[robot.disk.min_arg][1]], color='orange', label="Disk Center")
    # plt.scatter(robot.lidar.lidar_points[robot.disk.min_arg][0], robot.lidar.lidar_points[robot.disk.min_arg][1], color='orange', s=50, label="Closest Disk Point")


    # Plot the Circle patch around circle_center with radius R
    # disk = plt.Circle(robot.v_A, robot.turning_radius, color='green', fill=False, linewidth=1)
    # plt.scatter(robot.v_A[0], robot.v_A[1], color='red', s=10, label="Circle Center")
    # ax.add_patch(disk)

    # disk = plt.Circle([0,0], 8, color='green', fill=False, linewidth=1)
    # ax.add_patch(disk)

    # disk = plt.Circle([0,0], 8, color='green', fill=False, linewidth=1)
    # ax.add_patch(disk)


    plt.xlabel("x")
    plt.ylabel("y")
    # plt.title("Dubins Car Path with Simulated LiDAR and Obstacles")
    plt.axis("equal")
    # plt.legend()
    plt.grid(True)
    plt.show()


plotting_results()

# Данные для записи
robot_data = {
    "x": robot.x_path,
    "y": robot.y_path,
    "theta": robot.theta_path,
    "mode": robot.mode_path,
    "dR": robot.dR_path,
    "ddR": robot.ddR_path,
    "saturation": robot.sat_path,
    "second_part": robot.second_part_path,
    "sgn": robot.sgn_path,
    "u": robot.u_path,
    "d(t)": robot.d_path,
    "r": robot.r_path,
    "p": robot.p_path,
    "v(A)": robot.vA_path
}

# Запись данных в файл
with open(f"robot_data.json", "w") as json_file:
    json.dump(robot_data, json_file, indent=4)  # indent=4 делает вывод красивым