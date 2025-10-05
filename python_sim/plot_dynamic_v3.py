import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time as TIME

plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 18,
    'axes.labelsize': 18,
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'legend.fontsize': 12,
    'figure.titlesize': 18,
})

# Загрузка данных из JSON файла
with open('robot_data_dynamic_static_3obs_noise.json', 'r', encoding='utf-8') as json_file:
    data = json.load(json_file)

time_end = 10000
t_list = np.arange(0, time_end)
print(len(t_list))
robot_x = data["x"][1:time_end]
robot_y = data["y"][1:time_end]

# Данные для эллипсов
eps_1_x = data["eps_1_x"][1:time_end]
eps_1_y = data["eps_1_y"][1:time_end]
eps_1_rot = data["eps_1_rot"][1:time_end]
eps_2_x = data["eps_2_x"][1:time_end]
eps_2_y = data["eps_2_y"][1:time_end]
eps_2_rot = data["eps_2_rot"][1:time_end]
eps_3_x = data["eps_3_x"][1:time_end]
eps_3_y = data["eps_3_y"][1:time_end]
eps_3_rot = data["eps_3_rot"][1:time_end]

print(f"robot_x length: {len(robot_x)}")
print(f"eps_1_x length: {len(eps_1_x)}")
print(f"eps_2_x length: {len(eps_2_x)}")
print(f"eps_3_x length: {len(eps_3_x)}")

# Параметры эллипсов
a1, b1 = 60, 10    # Первый эллипс
a2, b2 = 60, 10    # Второй эллипс
a3, b3 = 60, 10    # Третий эллипс

def get_ellipse_points(x0, y0, a, b, theta):
    t_vals = np.linspace(0, 2 * np.pi, 100)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    x = x0 + a * np.cos(t_vals) * cos_t - b * np.sin(t_vals) * sin_t
    y = y0 + a * np.cos(t_vals) * sin_t + b * np.sin(t_vals) * cos_t
    return x, y

def get_equidistant_points(ellipse_data, d=1.0, num_points=500):
    x0, y0, a, b, theta = ellipse_data
    t = np.linspace(0, 2 * np.pi, num_points)

    x_ellipse = x0 + a * np.cos(t)
    y_ellipse = y0 + b * np.sin(t)

    dx = -a * np.sin(t)
    dy = b * np.cos(t)

    nx = dy
    ny = -dx

    norm = np.hypot(nx, ny)
    nx /= norm
    ny /= norm

    x_eq = x_ellipse + d * nx
    y_eq = y_ellipse + d * ny

    if theta != 0:
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        def rotate(x, y):
            x_new = (x - x0) * cos_t - (y - y0) * sin_t + x0
            y_new = (x - x0) * sin_t + (y - y0) * cos_t + y0
            return x_new, y_new

        x_ellipse, y_ellipse = rotate(x_ellipse, y_ellipse)
        x_eq, y_eq = rotate(x_eq, y_eq)

    return x_eq, y_eq

# Создание фигуры и осей
fig, ax = plt.subplots(figsize=(9, 9), dpi=150)

# Линии для эллипсов
ellipse1_line, = ax.plot([], [], color='black', linestyle='-', lw=2, label='Obstacles')
ellipse1_center, = ax.plot([], [], 'k+', markersize=6)
ellipse1_path, = ax.plot([], [], 'k--', lw=1, alpha=0.3, label='Obstacles\' Paths')

ellipse2_line, = ax.plot([], [], color='black', linestyle='-', lw=2)
ellipse2_center, = ax.plot([], [], 'k+', markersize=6)
ellipse2_path, = ax.plot([], [], 'k--', lw=1, alpha=0.3)

ellipse3_line, = ax.plot([], [], color='black', linestyle='-', lw=2)
ellipse3_center, = ax.plot([], [], 'k+', markersize=6)
ellipse3_path, = ax.plot([], [], 'k--', lw=1, alpha=0.3)

# Линии для эквидистант
equidist1_line, = ax.plot([], [], 'b--', lw=1.0, label='Equidistant')
equidist2_line, = ax.plot([], [], 'b--', lw=1.0)
equidist3_line, = ax.plot([], [], 'b--', lw=1.0)

# Линии для робота
robot_path_line, = ax.plot([], [], 'r-', lw=1.5, alpha=0.7, label='Robot Path', zorder=2)
robot_point, = ax.plot([], [], 'g*', markersize=10, label='Robot Pose', zorder=3)
robot_start_point, = ax.plot([], [], 'ko', markersize=3, label='Start', zorder=3)

# ax.set_xlabel(r"$x, m$")
# ax.set_ylabel(r"$y, m$")
# ax.set_title("Scenario 1")
# ax.legend()
ax.grid(True)
ax.set_aspect('equal', 'box')

# Вычисляем общие границы для всех кадров заранее
all_ellipse1_points_x = []
all_ellipse1_points_y = []
all_ellipse2_points_x = []
all_ellipse2_points_y = []
all_ellipse3_points_x = []
all_ellipse3_points_y = []

for i in range(len(robot_x)):
    x1, y1 = eps_1_x[i], eps_1_y[i]
    angle1 = eps_1_rot[i]
    ex1, ey1 = get_ellipse_points(x1, y1, a1, b1, angle1)
    all_ellipse1_points_x.extend(ex1)
    all_ellipse1_points_y.extend(ey1)
    
    x2, y2 = eps_2_x[i], eps_2_y[i]
    angle2 = eps_2_rot[i]
    ex2, ey2 = get_ellipse_points(x2, y2, a2, b2, angle2)
    all_ellipse2_points_x.extend(ex2)
    all_ellipse2_points_y.extend(ey2)

    x3, y3 = eps_3_x[i], eps_3_y[i]
    angle3 = eps_3_rot[i]
    ex3, ey3 = get_ellipse_points(x3, y3, a3, b3, angle3)
    all_ellipse3_points_x.extend(ex3)
    all_ellipse3_points_y.extend(ey3)

all_x = np.concatenate([robot_x, eps_1_x, eps_2_x, all_ellipse1_points_x, all_ellipse2_points_x, eps_3_x, all_ellipse3_points_x])
all_y = np.concatenate([robot_y, eps_1_y, eps_2_y, all_ellipse1_points_y, all_ellipse2_points_y, eps_3_y, all_ellipse3_points_y])

padding = 20

add_pad = (max(all_x) - min(all_x) - max(all_y) + min(all_y)) / 2

x_min, x_max = min(all_x) - padding, max(all_x) + padding
y_min, y_max = min(all_y) - padding - add_pad, max(all_y) + padding + add_pad

# Устанавливаем фиксированные границы
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_aspect('equal', 'box')

# Функция init для анимации
def init():
    ellipse1_line.set_data([], [])
    ellipse1_center.set_data([], [])
    ellipse1_path.set_data([], [])
    
    ellipse2_line.set_data([], [])
    ellipse2_center.set_data([], [])
    ellipse2_path.set_data([], [])

    ellipse3_line.set_data([], [])
    ellipse3_center.set_data([], [])
    ellipse3_path.set_data([], [])
    
    robot_path_line.set_data([], [])
    robot_point.set_data([], [])
    robot_start_point.set_data(robot_x[0], robot_y[0])

    equidist1_line.set_data([], [])
    equidist2_line.set_data([], [])
    equidist3_line.set_data([], [])

    return (
        ellipse1_line, ellipse1_center, ellipse1_path,
        ellipse2_line, ellipse2_center, ellipse2_path,
        ellipse3_line, ellipse3_center, ellipse3_path,
        robot_path_line, robot_point, robot_start_point,
        equidist1_line, equidist2_line, equidist3_line
    )

# Функция update для анимации
def update(frame):
    # === Эллипс 1 ===
    x1, y1 = eps_1_x[frame], eps_1_y[frame]
    angle1 = eps_1_rot[frame]
    ex1, ey1 = get_ellipse_points(x1, y1, a1, b1, angle1)
    ellipse1_line.set_data(ex1, ey1)
    ellipse1_center.set_data(x1, y1)
    ellipse1_path.set_data(eps_1_x[:frame+1], eps_1_y[:frame+1])

    # Эквидистанта 1
    ellipse_data1 = (x1, y1, a1, b1, angle1)
    eqx1, eqy1 = get_equidistant_points(ellipse_data1, d=5)  # d - смещение
    equidist1_line.set_data(eqx1, eqy1)

    # === Эллипс 2 ===
    x2, y2 = eps_2_x[frame], eps_2_y[frame]
    angle2 = eps_2_rot[frame]
    ex2, ey2 = get_ellipse_points(x2, y2, a2, b2, angle2)
    ellipse2_line.set_data(ex2, ey2)
    ellipse2_center.set_data(x2, y2)
    ellipse2_path.set_data(eps_2_x[:frame+1], eps_2_y[:frame+1])

    # Эквидистанта 2
    ellipse_data2 = (x2, y2, a2, b2, angle2)
    eqx2, eqy2 = get_equidistant_points(ellipse_data2, d=5)
    equidist2_line.set_data(eqx2, eqy2)

    # === Эллипс 3 ===
    x3, y3 = eps_3_x[frame], eps_3_y[frame]
    angle3 = eps_3_rot[frame]
    ex3, ey3 = get_ellipse_points(x3, y3, a3, b3, angle3)
    ellipse3_line.set_data(ex3, ey3)
    ellipse3_center.set_data(x3, y3)
    ellipse3_path.set_data(eps_3_x[:frame+1], eps_3_y[:frame+1])

    # Эквидистанта 3
    ellipse_data3 = (x3, y3, a3, b3, angle3)
    eqx3, eqy3 = get_equidistant_points(ellipse_data3, d=5)
    equidist3_line.set_data(eqx3, eqy3)

    # === Робот ===
    rx = robot_x[:frame+1]
    ry = robot_y[:frame+1]
    robot_path_line.set_data(rx, ry)
    robot_point.set_data(robot_x[frame], robot_y[frame])
    robot_start_point.set_data(robot_x[0], robot_y[0])

    return (
        ellipse1_line, ellipse1_center, ellipse1_path,
        ellipse2_line, ellipse2_center, ellipse2_path,
        ellipse3_line, ellipse3_center, ellipse3_path,
        robot_path_line, robot_point, robot_start_point,
        equidist1_line, equidist2_line, equidist3_line
    )

# Создание анимации
ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(robot_x),
    init_func=init,
    blit=True,
    interval=50,
    repeat=False
)

# Сохранение анимации в виде MP4
start_time = TIME.time()
# ani.save('static_3obs_noise.mp4', writer='ffmpeg', fps=60, dpi=150)
stop_time = TIME.time()
print(f"Animation saved in {(stop_time-start_time)/60:.2f} minutes")

# Отображение графика
plt.show()  