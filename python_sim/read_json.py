import json
import numpy as np
import matplotlib.pyplot as plt

time_end = 6600

with open('robot_data.json', 'r', encoding='utf-8') as json_file:
    data = json.load(json_file)

x = data["x"][1:]
y = data["y"][1:]
theta = data["theta"]
mode = data["mode"]
dR = data["dR"]
ddR = data["ddR"]
saturation = data["saturation"]
second_part = data["second_part"]
sgn = data["sgn"]
u = data["u"]
r = data["r"]
d = data["d(t)"]
v = data["v(A)"]
p = data["p"]


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


def plotting_results():
    fig, ax = plt.subplots()
    # Plot the Dubins car path
    plt.scatter(x[0], y[0], color='black', s=50, label="start point")
    plt.scatter(x[time_end-1], y[time_end-1], color='green', s=100, label="end point", marker='*')
    plt.plot(x[0:time_end-1], y[0:time_end-1], label="Dubins Car Path", color='red')
    # Plot obstacles
    plot_obstacles(obstacles)
    plot_equidistants(equidistants)

    # # Plot LiDAR points
    # # plt.scatter(robot.lidar.lidar_points[:,0], robot.lidar.lidar_points[:,1], color='red', s=1, label="LiDAR Rays")
    # # print(robot.lidar.lidar_closest_point)
    
    # # Plot closest point and circle center
    # plt.scatter(robot.lidar.lidar_closest_point[0], robot.lidar.lidar_closest_point[1], color='orange', s=50, label="Closest Point")
    # # plt.plot([robot.disk.disk_center[0], robot.lidar.lidar_points[robot.disk.min_arg][0]], [robot.disk.disk_center[1], robot.lidar.lidar_points[robot.disk.min_arg][1]], color='orange', label="Stop")
    # plt.scatter(robot.lidar.lidar_points[robot.disk.min_arg][0], robot.lidar.lidar_points[robot.disk.min_arg][1], color='orange', s=50, label="Closest Point")


    # # Plot the Circle patch around circle_center with radius R
    # disk = plt.Circle(robot.v_A, robot.turning_radius, color='green', fill=False, linewidth=1)
    # plt.scatter(robot.v_A[0], robot.v_A[1], color='red', s=10, label="Circle Center")
    # ax.add_patch(disk)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Dubins Car Path with Simulated LiDAR and Obstacles")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()


with open(f'data_{time_end}', 'w') as f:
    for i in range(time_end):
        line_1 = f'Mode {mode[i]} | {round(u[i], 4):>6} = 0.025 * sgn({round(ddR[i], 4):>7} + 0.025 * sat({round(dR[i], 4):>7})) '
        if mode[i] == "C":
            line_2 = f'r={r[i]} d(t)={round(d[i], 3):>6} p(t)={p[i]} e(theta)={round(theta[i], 3):>6}\n'
        else:
            line_2 = f'r={r[i]} v(A)={v[i]} p(t)= NONE e(theta)={round(theta[i], 3):>6}\n'
        f.write(line_1)
        f.write(line_2)

plotting_results()