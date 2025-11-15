import math
import numpy as np
import matplotlib.pyplot as plt


def normalize_angle_minus_pi_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class DubinsCar:
    def __init__(self, x=0, y=0, theta=0, max_speed=0.5, k_p=2.0):
        self.x = x  # позиция X
        self.y = y  # позиция Y
        self.theta = theta  # ориентация робота (в радианах)
        self.max_speed = max_speed  # максимальная линейная скорость
        self.k_p = k_p  # коэффициент П-регулятора для поворота

    def step(self, target_azimuth, dt=0.1):
        """Один шаг симуляции"""
        # Ошибка по углу
        error_angle = normalize_angle_minus_pi_to_pi(target_azimuth - self.theta)

        # Управление угловой скоростью (П-регулятор)
        angular_velocity = self.k_p * error_angle

        # Обновляем ориентацию
        self.theta += angular_velocity * dt

        # Двигаемся вперёд
        self.x += self.max_speed * math.cos(self.theta) * dt
        self.y += self.max_speed * math.sin(self.theta) * dt

        return self.x, self.y, self.theta


# === Симуляция ===
if __name__ == "__main__":
    # Начальные параметры
    robot = DubinsCar(x=0, y=0, theta=0)  # начальное направление — вправо
    target_azimuth = math.radians(-60)     # цель на 60 градусов от оси X

    trajectory = []  # для хранения траектории

    # Запускаем симуляцию на N шагов
    for _ in range(200):
        x, y, theta = robot.step(target_azimuth)
        trajectory.append((x, y, theta))

    # Конвертируем в массивы для отрисовки
    trajectory = np.array(trajectory)

    # Отрисовка
    plt.figure(figsize=(8, 8))
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Траектория')
    plt.quiver(trajectory[:, 0], trajectory[:, 1],
               np.cos(trajectory[:, 2]), np.sin(trajectory[:, 2]),
               color='r', scale=20, label='Ориентация робота')

    # Целевой азимут
    length = 5
    plt.plot([0, length * math.cos(target_azimuth)], [0, length * math.sin(target_azimuth)],
             'g--', label='Целевой азимут')

    plt.axis('equal')
    plt.title("Симуляция движения Dubins Car по заданному азимуту")
    plt.legend()
    plt.grid(True)
    plt.show()