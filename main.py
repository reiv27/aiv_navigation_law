import json
import numpy as np
import time as TIME
import matplotlib.pyplot as plt

from robot import *
from vector_operations import find_vector_with_dir

start_time = TIME.time()


dt = 1                             # Time step
time_end = 4750                    # Simulation time
time = np.arange(0, time_end, dt)  # Total simulation time
d = 5                              # Distance from equididstant to objects

robot = DubinsCar(0.10, 0.025, 5, d)
robot.init_pose(43, 20, np.pi/2)


# Obstacles and equidistant
obs_1 = np.array([[50, 10], [50, 65]], dtype=np.float32)
obs_2 = np.array([[50, 65], [0, 65]], dtype=np.float32)
obs_3 = np.array([[0, 65],  [0, 10]], dtype=np.float32)
obs_4 = np.array([[0, 10],   [50, 10]], dtype=np.float32)

equid_1 = np.array([[45, 15], [45, 60]], dtype=np.float32)
equid_2 = np.array([[45, 60], [5, 60]], dtype=np.float32)
equid_3 = np.array([[5, 60], [5, 15]], dtype=np.float32)
equid_4 = np.array([[5, 15], [45, 15]], dtype=np.float32)

# obstacles = np.array([obs_1, obs_2])
# equidistants = np.array([equid_1, equid_2])
obstacles = np.array([obs_1, obs_2, obs_3, obs_4])
equidistants = np.array([equid_1, equid_2, equid_3, equid_4])

def plot_obstacles(obstacles):
    for obstacle in obstacles:
        plt.plot([obstacle[0][0], obstacle[1][0]], [obstacle[0][1], obstacle[1][1]], color='black')


def plot_equidistants(equidistants):
    for equidistant in equidistants:
        plt.plot([equidistant[0][0], equidistant[1][0]], [equidistant[0][1], equidistant[1][1]], color='blue')

# Simulate
for j, t in enumerate(time):
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
    plot_equidistants(equidistants)

    # Plot LiDAR points
    # plt.scatter(robot.lidar.lidar_points[:,0], robot.lidar.lidar_points[:,1], color='red', s=1, label="LiDAR Rays")
    # print(robot.lidar.lidar_closest_point)
    
    # Plot closest point and circle center
    plt.scatter(robot.lidar.lidar_closest_point[0], robot.lidar.lidar_closest_point[1], color='orange', s=50, label="Closest Point")
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


plotting_results()

# Данные для записи
robot_data = {
    "x": robot.x_path,
    "y": robot.x_path,
    "mode": robot.mode_path,
    "dR": robot.dR_path,
    "ddR": robot.ddR_path,
    "saturation": robot.sat_path,
    "second_part": robot.second_part_path,
    "sgn": robot.sgn_path,
    "u": robot.u_path
}

for data in robot_data:
    print(len(data))

# Запись данных в файл
with open("robot_data.json", "w") as json_file:
    json.dump(robot_data, json_file, indent=4)  # indent=4 делает вывод красивым