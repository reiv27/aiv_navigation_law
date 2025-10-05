import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time as TIME


with open('robot_data_dynamic.json', 'r', encoding='utf-8') as json_file:
    data = json.load(json_file)
time_end = data["t"]
t_list = np.arange(0, time_end)
print(len(t_list))
robot_x = data["x"][1:]
robot_y = data["y"][1:]
eps_x = data["eps_1_x"][1:]
eps_y = data["eps_1_y"][1:]
eps_rot = data["eps_1_rot"][1:]

a = 60         
b = 10        
theta = 0
a2 = a + 10
b2 = b + 10

def get_ellipse_points(x0, y0, a, b, theta):
    t_vals = np.linspace(0, 2 * np.pi, 100)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    x = x0 + a * np.cos(t_vals) * cos_t - b * np.sin(t_vals) * sin_t
    y = y0 + a * np.cos(t_vals) * sin_t + b * np.sin(t_vals) * cos_t
    return x, y

fig, ax = plt.subplots(figsize=(16, 9), dpi=150)

ellipse_line1, = ax.plot([], [], 'k-', lw=2, label='Obstacle')
ellipse_center1, = ax.plot([], [], 'k+', markersize=6)

ellipse_line2, = ax.plot([], [], 'b-', lw=1, label='Equidistant')
ellipse_center2, = ax.plot([], [], 'b+', markersize=1)

robot_path_line, = ax.plot([], [], 'r-', lw=1, alpha=0.7, label='Robot Path')
robot_point, = ax.plot([], [], 'g*', markersize=10, label='Robot Pose')
robot_start_point, = ax.plot([], [], 'ko', markersize=6, label='Start')

ellipse_path, = ax.plot([], [], 'k--', lw=1, alpha=0.5, label='Obstacle Path')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Dubins Car Path")
ax.legend()
ax.grid(True)
ax.set_aspect('equal', 'box')

all_x = np.concatenate([robot_x, eps_x])
all_y = np.concatenate([robot_y, eps_y])
padding = 10
ax.set_xlim(all_x.min() - padding, all_x.max() + padding)
ax.set_ylim(all_y.min() - padding, all_y.max() + padding)

def init():
    ellipse_line1.set_data([], [])
    ellipse_center1.set_data([], [])
    ellipse_line2.set_data([], [])
    ellipse_center2.set_data([], [])

    robot_path_line.set_data([], [])
    robot_point.set_data([], [])
    robot_start_point.set_data(robot_x[0], robot_y[0])

    ellipse_path.set_data([], [])

    return (
        ellipse_line1, ellipse_center1,
        ellipse_line2, ellipse_center2,
        robot_path_line, robot_point, robot_start_point,
        ellipse_path,
    )

def update(frame):
    x0 = eps_x[frame]
    y0 = eps_y[frame]
    angle = eps_rot[frame]

    ex1, ey1 = get_ellipse_points(x0, y0, a, b, angle)
    ellipse_line1.set_data(ex1, ey1)
    ellipse_center1.set_data(x0, y0)

    ex2, ey2 = get_ellipse_points(x0, y0, a2, b2, angle)
    ellipse_line2.set_data(ex2, ey2)
    ellipse_center2.set_data(x0, y0)

    ellipse_path.set_data(eps_x[:frame+1], eps_y[:frame+1])

    rx = robot_x[:frame+1]
    ry = robot_y[:frame+1]
    robot_path_line.set_data(rx, ry)
    robot_point.set_data(robot_x[frame], robot_y[frame])
    robot_start_point.set_data(robot_x[0], robot_y[0])

    return (
        ellipse_line1, ellipse_center1,
        ellipse_line2, ellipse_center2,
        robot_path_line, robot_point, robot_start_point,
        ellipse_path,
    )

ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(robot_x),
    init_func=init,
    blit=True,
    interval=50,
    repeat=False
)

start_time = TIME.time()
ani.save('dynamic_obstacle_new_rot.mp4', writer='ffmpeg', fps=60, dpi=150)
stop_time = TIME.time()
print((stop_time-start_time) / 60)

plt.show()