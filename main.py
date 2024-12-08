import json
import numpy as np
import time as TIME
import matplotlib.pyplot as plt

from robot import *
from vector_operations import find_vector_with_dir

start_time = TIME.time()


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
# # equidistants = np.array([equid_1, equid_2, equid_3, equid_4])

# Obstacles 3
obs_1 = np.array([[0, 0], [70, 50]], dtype=np.float64)
obs_2 = np.array([[70, 50], [150, 50]], dtype=np.float64)
obs_3 = np.array([[150, 50], [220, 0]], dtype=np.float64)
obs_4 = np.array([[220, 0], [220, 140]], dtype=np.float64)
obs_5 = np.array([[220, 140], [150, 90]], dtype=np.float64)
obs_6 = np.array([[150, 90], [70, 90]], dtype=np.float64)
obs_7 = np.array([[70, 90], [0, 140]], dtype=np.float64)
obs_8 = np.array([[0, 140], [0, 0]], dtype=np.float64)

obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8])
# obstacles = np.array([obs_4, obs_5])
# equidistants = np.array([equid_1, equid_2, equid_3, equid_4])


# Obstacles 4
# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obs_1 = np.array([[0, 0], [5, 0]], dtype=np.float64)
# obs_2 = np.array([[5, 0], [5, 5]], dtype=np.float64)
# obs_3 = np.array([[5, 5], [0, 5]], dtype=np.float64)
# obs_4 = np.array([[0, 5], [0, 0]], dtype=np.float64)

# obstacles = np.array([obs_1, obs_2, obs_3, obs_4, obs_5, obs_6, obs_7, obs_8])
# # equidistants = np.array([equid_1, equid_2, equid_3, equid_4])



def plot_obstacles(obstacles):
    for obstacle in obstacles:
        plt.plot([obstacle[0][0], obstacle[1][0]], [obstacle[0][1], obstacle[1][1]], color='black')


def plot_equidistants(equidistants):
    for equidistant in equidistants:
        plt.plot([equidistant[0][0], equidistant[1][0]], [equidistant[0][1], equidistant[1][1]], color='blue')


dt = 1                             # Time step
time_end = 20000                   # Simulation time
time = np.arange(0, time_end, dt)  # Total simulation time
d = 8                              # Distance from equididstant to objects

robot = DubinsCar(0.1, 0.025, 5, d)
robot.init_pose(12, 50, np.pi/2)

# Simulate
for t in time:
    # Update position based on Dubins car kinematics
    robot.update_pose(dt)

    # LiDAR scan
    robot.lidar_scan(obstacles)

    # Update disk rays
    robot.update_disk_center()
    robot.update_disk_rays()

    # Check robot state
    robot.switch_mode()

    # Calculate U
    robot.calculate_u()
    # if t > 321:
    #     break
    # if robot.state:
    #     break
    robot.update_orientation(dt)

    # Print some data
    # robot.telemetry_output()
    

def plotting_results():
    fig, ax = plt.subplots()
    # Plot the Dubins car path
    plt.scatter(robot.x_path[0], robot.y_path[0], color='black', s=50, label="start point")
    plt.scatter(robot.x_path[-1], robot.y_path[-1], color='green', s=100, label="end point", marker='*')
    plt.plot(robot.x_path, robot.y_path, label="Dubins Car Path", color='red')
    # Plot obstacles
    plot_obstacles(obstacles)
    # plot_equidistants(equidistants)

    # Plot LiDAR points
    plt.scatter(robot.lidar.lidar_points[:,0], robot.lidar.lidar_points[:,1], color='red', s=1, label="LiDAR Rays")
    # print(robot.lidar.lidar_closest_point)
    
    # Plot closest point and circle center
    plt.scatter(robot.lidar.lidar_closest_point[0], robot.lidar.lidar_closest_point[1], color='red', s=50, label="Closest Point")
    # plt.plot([robot.disk.disk_center[0], robot.lidar.lidar_points[robot.disk.min_arg][0]], [robot.disk.disk_center[1], robot.lidar.lidar_points[robot.disk.min_arg][1]], color='orange', label="Stop")
    plt.scatter(robot.lidar.lidar_points[robot.disk.min_arg][0], robot.lidar.lidar_points[robot.disk.min_arg][1], color='orange', s=50, label="Closest Point")


    # Plot the Circle patch around circle_center with radius R
    disk = plt.Circle(robot.v_A, robot.turning_radius, color='green', fill=False, linewidth=1)
    plt.scatter(robot.v_A[0], robot.v_A[1], color='red', s=10, label="Circle Center")
    ax.add_patch(disk)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Dubins Car Path with Simulated LiDAR and Obstacles")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()


# print(len(robot.x_path))
# print(len(robot.y_path))
# print(len(robot.theta_path))

# print(len(robot.mode_path))
# print(len(robot.dR_path))
# print(len(robot.ddR_path))
# print(len(robot.sat_path))
# print(len(robot.second_part_path))
# print(len(robot.sgn_path))
# print(len(robot.u_path))


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