import numpy as np
import time as TIME
import matplotlib.pyplot as plt

from robot import *
from vector_operations import find_vector_with_dir

start_time = TIME.time()


dt = 0.1                           # Time step
time_end = 60                    # Simulation time
time = np.arange(0, time_end, dt)  # Total simulation time
d = 5                              # Distance from equididstant to objects

# # linear_linelocityechicle parameters
# linear_velocity = 0.10             # Constant forward speed linear_linelocity
# angular_velocity = 0.025           # Constant rotate speed w
# R_min = 5                          # Radius of the circle

robot = DubinsCar(0.10, 0.025, 5, d)

# Init pose
# x = 43.0
# y = 35.0
# theta = -np.pi / 2                  # Starting orientation (90 degrees, facing upwards)
# e = np.array([[np.cos(theta)],
#               [np.sin(theta)]])

# x_path = [x]    # Store the x
# y_path = [y]    # Store the y

robot.init_pose(43, 20, np.pi/2)
print(robot.theta)
print(robot.e)



# # Lidar parameters
# lidar_range = 20                   # Maximum range of LiDAR
# lidar_resolution = 360             # Number of rays in 360 degrees
# lidar_points = np.zeros(           # Coordinates of all rays
#     (lidar_resolution-1, 2),
#     dtype=np.float32
# )
# lidar_distances = np.zeros(        # Value of all rays' distances
#     lidar_resolution-1,
#     dtype=np.float32
# )
# angles_of_rays = np.linspace(0, 2 * np.pi, lidar_resolution)


# disk_rays_lengths = np.zeros(      # Value of all rays' distances
#     lidar_resolution-1,
#     dtype=np.float32
# )


# #################################
# ### Obstacles and equidistant ###
# #################################

obs_1 = np.array([[50, 10], [50, 65]], dtype=np.float32)
obs_2 = np.array([[50, 65], [0, 65]], dtype=np.float32)
# obs_3 = np.array([[0, 65],  [0, 10]], dtype=np.float32)
# obs_4 = np.array([[0, 10],   [50, 10]], dtype=np.float32)

equid_1 = np.array([[45, 15], [45, 60]], dtype=np.float32)
equid_2 = np.array([[45, 60], [5, 60]], dtype=np.float32)
# equid_3 = np.array([[5, 60], [5, 15]], dtype=np.float32)
# equid_4 = np.array([[5, 15], [45, 15]], dtype=np.float32)

obstacles = np.array([obs_1, obs_2])
equidistants = np.array([equid_1, equid_2])

def plot_obstacles(obstacles):
    for obstacle in obstacles:
        # plt.plot([obstacle[0][0], obstacle[1][0]], [obstacle[0][1], obstacle[1][1]], color='black', label="Obstacles")
        plt.plot([obstacle[0][0], obstacle[1][0]], [obstacle[0][1], obstacle[1][1]], color='black')


def plot_equidistants(equidistants):
    for equidistant in equidistants:
        # plt.plot([equidistant[0][0], equidistant[1][0]], [equidistant[0][1], equidistant[1][1]], color='blue', label="Equidistant")
        plt.plot([equidistant[0][0], equidistant[1][0]], [equidistant[0][1], equidistant[1][1]], color='blue')


# def find_intersection(vector_1, vector_2):
#     # Compute direction vectors
#     d1 = vector_1[1] - vector_1[0]
#     d2 = vector_2[1] - vector_2[0]

#     # Set up the linear system for intersection
#     A = np.array([d1, -d2]).T
#     b = vector_2[0] - vector_1[0]

#     # Solve for intersection if not parallel
#     if np.linalg.matrix_rank(A) == 2:
#         t, s = np.linalg.solve(A, b)
#         if 0 <= t <= 1 and 0 <= s <= 1:
#             return vector_1[0] + t * d1


# Simulate
for j, t in enumerate(time):
    # Update position based on Dubins car kinematics
    robot.update_pose(dt)

    # LiDAR scan
    robot.lidar_scan(obstacles)
    # print(robot.lidar.closest_distance, robot.lidar.lidar_closest_point)

#     # Finding center point of circle on equidistant
#     # Calculate the direction linear_linelocityector and normalize it
#     dir = np.array([x, y]) - lidar_closest_point
#     unit_dir = dir / np.linalg.norm(dir)
#     # Calculate the new endpoint at the specified length
#     disk_center = lidar_closest_point + unit_dir * (d + R_min)
    # disk_center = find_vector_with_dir(
    #     np.array([robot.x, robot.y]),
    #     robot.lidar.closest_point,
    #     d + robot.R_min
    # )

    robot.update_disk_center()


#     target_index = np.argmin(lidar_distances)
#     start = target_index - 10
#     end = target_index + 10
#     interval = np.arange(start, end, 1)
#     for i in range(lidar_resolution-1):
#         if i in interval:
#             disk_rays_lengths[i] = lidar_range + 10
#             continue
#         disk_rays_lengths[i] = np.linalg.norm(lidar_points[i] - disk_center)
#         # print(i, disk_rays_lengths[i])

    robot.update_disk_rays()
#     min_disk_length = np.min(disk_rays_lengths)
#     min_arg = np.argmin(disk_rays_lengths)
#     # print(min_disk_length, np.linalg.norm(disk_center-lidar_closest_point))

#     r = np.array([x, y])
#     v = disk_center
#     p = np.array([lidar_closest_point[0], lidar_closest_point[1]])

#     if min_disk_length < np.linalg.norm(disk_center-lidar_closest_point):
#         # print(min_arg, min_disk_length, np.linalg.norm(disk_center-lidar_closest_point), t)
#         # print('BREAK')
#         # print(disk_center, lidar_closest_point, lidar_points[min_arg], np.array([x, y]))
#         # print(is_point_in_angle(disk_center, lidar_closest_point, lidar_points[min_arg], np.array([x, y])))

#         if is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], r):
#             while True:
#                 print("Mode G")
#                 dR = R_min - np.linalg.norm(r - v)
#                 ddR = (((v-r) / np.linalg.norm(v-r)) @ e)[0]
#                 u_input = angular_velocity * np.sign(ddR + 1 * 0.025 * dR)
#                 print(u_input)
#                 theta += u_input * dt
#                 # theta += 0.025 * dt
#                 e = np.array([[np.cos(theta)], [np.sin(theta)]])
#                 x += linear_velocity * e[0][0] * dt
#                 y += linear_velocity * e[1][0] * dt
#                 # Store the path
#                 x_path.append(x)
#                 y_path.append(y)
#                 if not is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], np.array([x, y])):
#                     break
#                 # print(x, y, theta)
#             # is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], np.array([x, y]))

#     print(f"Mode C; {round(t, 1)}/{len(time) * dt}")
#     # Calculate U(t) for Mode C
#     dR = closest_distance - d
#     ddR = (((r-p) / np.linalg.norm(r-p)) @ e)[0]
#     u_input = angular_velocity * np.sign(ddR + 1 * 0.025 * dR)

#     theta += u_input * dt
#     e = np.array([[np.cos(theta)], [np.sin(theta)]])

#     # print(f'd_R = {dR}, dd_R = {ddR}')
#     # print(f'w(t) = {u_input}')
#     # print(f'theta = {theta}', end="\n")
#     # print('')


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
    plt.scatter(robot.lidar.lidar_points[:,0], robot.lidar.lidar_points[:,1], color='red', s=1, label="LiDAR Rays")
    # print(robot.lidar.lidar_closest_point)
    
    # Plot closest point and circle center
    plt.scatter(robot.lidar.lidar_closest_point[0], robot.lidar.lidar_closest_point[1], color='orange', s=50, label="Closest Point")
    plt.plot([robot.disk.disk_center[0], robot.lidar.lidar_points[robot.disk.min_arg][0]], [robot.disk.disk_center[1], robot.lidar.lidar_points[robot.disk.min_arg][1]], color='orange', label="Stop")


    # Plot the Circle patch around circle_center with radius R
    disk = plt.Circle(robot.disk.disk_center, robot.turning_radius, color='green', fill=False, linewidth=1)
    plt.scatter(robot.disk.disk_center[0], robot.disk.disk_center[1], color='red', s=10, label="Circle Center")
    ax.add_patch(disk)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Dubins Car Path with Simulated LiDAR and Obstacles")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()


# # target_index = np.argmin(lidar_distances)
# # start = target_index - 10
# # end = target_index + 10
# # new_id = np.arange(start, end, 1)
# # for i in new_id:
# #     print(f'{i} : {lidar_distances[i]}')

# stop_time = TIME.time()
# print(stop_time - start_time)

plotting_results()