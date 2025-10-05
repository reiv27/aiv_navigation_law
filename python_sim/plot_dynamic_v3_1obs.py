import json
import numpy as np
import matplotlib.pyplot as plt

# Загрузка данных из JSON файла
with open('robot_data_dynamic.json', 'r', encoding='utf-8') as json_file:
    data = json.load(json_file)

time_end = 18000
robot_x = data["x"][1:time_end]
robot_y = data["y"][1:time_end]

# Данные для эллипсов
eps_1_x = data["eps_1_x"][1:time_end]
eps_1_y = data["eps_1_y"][1:time_end]
eps_1_rot = data["eps_1_rot"][1:time_end]

print(f"robot_x length: {len(robot_x)}")
print(f"eps_1_x length: {len(eps_1_x)}")

# Параметры эллипсов
a1, b1 = 60, 10  # Первый эллипс

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
fig, ax = plt.subplots(figsize=(16, 9), dpi=150)

# Последнее положение эллипса и робота
frame = len(robot_x) - 1

# === Эллипс 1 ===
x1, y1 = eps_1_x[frame], eps_1_y[frame]
angle1 = eps_1_rot[frame]
ex1, ey1 = get_ellipse_points(x1, y1, a1, b1, angle1)
ax.plot(ex1, ey1, color='black', linestyle='-', lw=2, label='Obstacle')

# Центр эллипса
ax.plot(x1, y1, 'k+', markersize=10)

# Эквидистанта 1
ellipse_data1 = (x1, y1, a1, b1, angle1)
eqx1, eqy1 = get_equidistant_points(ellipse_data1, d=10)  # d - смещение
ax.plot(eqx1, eqy1, 'b--', lw=1.5, label='Equidistant')

# Траектория робота
ax.plot(robot_x, robot_y, 'r-', lw=1, alpha=0.7, label='Robot Path')
ax.plot(robot_x[-1], robot_y[-1], 'g*', markersize=10, label='Robot Pose')
ax.plot(robot_x[0], robot_y[0], 'ko', markersize=6, label='Start')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Static Plot of Robot and Obstacle")
ax.legend()
ax.grid(True)
ax.set_aspect('equal', 'box')

# Автоматические границы
all_x = np.concatenate([robot_x, [x1], ex1])
all_y = np.concatenate([robot_y, [y1], ey1])

padding = 20
x_min, x_max = min(all_x) - padding, max(all_x) + padding
y_min, y_max = min(all_y) - padding, max(all_y) + padding
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)

# Сохранение как изображение (опционально)
# plt.savefig('static_plot.png')

# Отображение графика
plt.show()