import numpy as np
import time as TIME
import matplotlib.pyplot as plt

import robot

start_time = TIME.time()


dt = 0.1                           # Time step
time_end = 50                    # Simulation time
time = np.arange(0, time_end, dt)  # Total simulation time
d = 5                              # Distance from equididstant to objects

# linear_linelocityechicle parameters
linear_velocity = 0.10             # Constant forward speed linear_linelocity
angular_velocity = 0.025           # Constant rotate speed w
R_min = 5                          # Radius of the circle

# Init pose
x = 43.0
y = 20.0
theta = np.pi / 2                  # Starting orientation (90 degrees, facing upwards)
e = np.array([[np.cos(theta)],
              [np.sin(theta)]])

x_path = [x]    # Store the x
y_path = [y]    # Store the y

# Lidar parameters
lidar_range = 20                   # Maximum range of LiDAR
lidar_resolution = 360             # Number of rays in 360 degrees
lidar_points = np.zeros(           # Coordinates of all rays
    (lidar_resolution-1, 2),
    dtype=np.float32
)
lidar_distances = np.zeros(        # Value of all rays' distances
    lidar_resolution-1,
    dtype=np.float32
)
angles_of_rays = np.linspace(0, 2 * np.pi, lidar_resolution)


disk_rays_lengths = np.zeros(      # Value of all rays' distances
    lidar_resolution-1,
    dtype=np.float32
)


#################################
### Obstacles and equidistant ###
#################################

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


# Simulate
for j, t in enumerate(time):
    # print(j, t)

    # Update position based on Dubins car kinematics
    x += linear_velocity * e[0][0] * dt
    y += linear_velocity * e[1][0] * dt
    # Store the path
    x_path.append(x)
    y_path.append(y)

    # LiDAR scan
    for i, angle in enumerate(angles_of_rays[0:-1]):
        # Add ray's coridnate in list
        lidar_data_x = x + lidar_range * np.cos(theta + angle)
        lidar_data_y = y + lidar_range * np.sin(theta + angle)
        lidar_points[i][0] = lidar_data_x
        lidar_points[i][1] = lidar_data_y

        # Find intersection with obstacles
        for obs in obstacles:
            intersection = find_intersection(np.array([[x, y], [lidar_data_x, lidar_data_y]]), obs)
            # Rewrite point if it has intersection with obstacle
            if intersection is not None:
                lidar_points[i][0] = intersection[0]
                lidar_points[i][1] = intersection[1]

        # Calculate distance for ray
        lidar_distances[i] = np.linalg.norm(lidar_points[i] - [x, y])

    
    # Finding closest point for robot on objects
    closest_distance = np.min(lidar_distances)
    lidar_closest_point = lidar_points[np.argmin(lidar_distances)]

    # # Finding center point of circle on equidistant
    # # Calculate the direction linear_linelocityector and normalize it
    # dir = np.array([x, y]) - lidar_closest_point
    # unit_dir = dir / np.linalg.norm(dir)
    # # Calculate the new endpoint at the specified length
    # disk_center = lidar_closest_point + unit_dir * (d + R_min)


    # target_index = np.argmin(lidar_distances)
    # start = target_index - 10
    # end = target_index + 10
    # interval = np.arange(start, end, 1)
    # for i in range(lidar_resolution-1):
    #     if i in interval:
    #         disk_rays_lengths[i] = lidar_range + 10
    #         continue
    #     disk_rays_lengths[i] = np.linalg.norm(lidar_points[i] - disk_center)
    #     # print(i, disk_rays_lengths[i])

    # min_disk_length = np.min(disk_rays_lengths)
    # min_arg = np.argmin(disk_rays_lengths)
    # # print(min_disk_length, np.linalg.norm(disk_center-lidar_closest_point))

    # r = np.array([x, y])
    # v = disk_center
    # p = np.array([lidar_closest_point[0], lidar_closest_point[1]])

    # if min_disk_length < np.linalg.norm(disk_center-lidar_closest_point):
    #     # print(min_arg, min_disk_length, np.linalg.norm(disk_center-lidar_closest_point), t)
    #     # print('BREAK')
    #     # print(disk_center, lidar_closest_point, lidar_points[min_arg], np.array([x, y]))
    #     # print(is_point_in_angle(disk_center, lidar_closest_point, lidar_points[min_arg], np.array([x, y])))

    #     if is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], r):
    #         while True:
    #             print("Mode G")
    #             dR = R_min - np.linalg.norm(r - v)
    #             ddR = (((v-r) / np.linalg.norm(v-r)) @ e)[0]
    #             u_input = angular_velocity * np.sign(ddR + 1 * 0.025 * dR)
    #             print(u_input)
    #             theta += u_input * dt
    #             # theta += 0.025 * dt
    #             e = np.array([[np.cos(theta)], [np.sin(theta)]])
    #             x += linear_velocity * e[0][0] * dt
    #             y += linear_velocity * e[1][0] * dt
    #             # Store the path
    #             x_path.append(x)
    #             y_path.append(y)
    #             if not is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], np.array([x, y])):
    #                 break
    #             # print(x, y, theta)
    #         # is_point_in_angle(v, lidar_closest_point, lidar_points[min_arg], np.array([x, y]))

    # print(f"Mode C; {round(t, 1)}/{len(time) * dt}")
    # # Calculate U(t) for Mode C
    # dR = closest_distance - d
    # ddR = (((r-p) / np.linalg.norm(r-p)) @ e)[0]
    # u_input = angular_velocity * np.sign(ddR + 1 * 0.025 * dR)

    # theta += u_input * dt
    # e = np.array([[np.cos(theta)], [np.sin(theta)]])

    # # print(f'd_R = {dR}, dd_R = {ddR}')
    # # print(f'w(t) = {u_input}')
    # # print(f'theta = {theta}', end="\n")
    # # print('')


def plotting_results():
    fig, ax = plt.subplots()
    # Plot the Dubins car path
    plt.scatter(x_path[0], y_path[0], color='black', s=50, label="start point")
    plt.scatter(x_path[-1], y_path[-1], color='green', s=100, label="end point", marker='*')
    plt.plot(x_path, y_path, label="Dubins Car Path", color='red')
    # Plot obstacles
    plot_obstacles(obstacles)
    plot_equidistants(equidistants)

    # Plot LiDAR points
    # plt.scatter(lidar_points[:,0], lidar_points[:,1], color='orange', s=10, label="LiDAR Rays")

    # Plot closest point and circle center
    plt.scatter(lidar_closest_point[0], lidar_closest_point[1], color='orange', s=50, label="Closest Point")
    # plt.plot([disk_center[0], lidar_points[min_arg][0]], [disk_center[1], lidar_points[min_arg][1]], color='orange', label="Stop")


    # # Plot the Circle patch around circle_center with radius R
    # disk = plt.Circle(disk_center, R_min, color='green', fill=False, linewidth=1)
    # plt.scatter(disk_center[0], disk_center[1], color='red', s=10, label="Circle Center")
    # ax.add_patch(disk)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Dubins Car Path with Simulated LiDAR and Obstacles")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()


# target_index = np.argmin(lidar_distances)
# start = target_index - 10
# end = target_index + 10
# new_id = np.arange(start, end, 1)
# for i in new_id:
#     print(f'{i} : {lidar_distances[i]}')

stop_time = TIME.time()
print(stop_time - start_time)

plotting_results()