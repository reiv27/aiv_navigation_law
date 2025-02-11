import numpy as np
import vector_operations as vec_ops

class LiDAR:
    def __init__(self, lidar_range, lidar_resolution) -> None:
        self.lidar_range = lidar_range            # Maximum range of LiDAR
        self.lidar_resolution = lidar_resolution  # Number of rays in 360 degrees
        self.lidar_points = np.zeros(             # Coordinates of all rays
            (lidar_resolution-1, 2),
            dtype=np.float64
        )
        self.lidar_distances = np.zeros(          # Value of all rays' distances
            lidar_resolution-1,
            dtype=np.float64
        )
        self.angles_of_rays = np.linspace(0, 2 * np.pi, lidar_resolution)
        self.closest_distance = 0
        self.lidar_closest_point = np.array([0, 0])

    def simulate_lidar_measurement(self, accuracy):
        # Рассчитываем стандартное отклонение шума
        noise_std = accuracy * self.lidar_distances
        # Генерируем гауссовский шум
        noise = np.random.normal(0, noise_std, size=self.lidar_distances.shape)
        # Возвращаем измерения с шумом
        self.lidar_distances += noise

    def sma(self, core):
        filtered_data = np.zeros(
            self.lidar_resolution-1,
            dtype=np.float64
        )

        for i, distance in enumerate(self.lidar_distances):
            if i < core-1:
                sum = 0
                for j in range(i+1-core, i+1):
                    sum += self.lidar_distances[j]
                filtered_data[i] = sum / core
                continue

            filtered_data[i] = np.sum(self.lidar_distances[i+1-core:i+1]) / core

        return filtered_data


class TurningDisk:
    def __init__(self, R, resolution):
        self.R = R
        self.disk_center = np.array([0.0, 0.0])
        self.disk_rays_lengths = np.zeros( 
            resolution-1,
            dtype=np.float64
        )
        self.min_disk_length = 0
        self.min_arg = 0

class DubinsCar:

    mode_First = 0
    mode_Main = 1
    mode_C = 0
    mode_G = 1
    goal = 0


    def __init__(self, linear_velocity, angular_velocity, turning_radius, d) -> None:
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.R_min = linear_velocity / angular_velocity
        # self.turning_radius = turning_radius
        self.turning_radius = self.R_min
        self.d = d

        self.x = 0
        self.y = 0
        self.theta = 0
        self.e = np.array([
            [np.cos(self.theta)],
            [np.sin(self.theta)]
        ])
        
        self.lidar = LiDAR(50, 360)
        self.disk = TurningDisk(turning_radius, 360)
        
        self.state_global = self.mode_First
        self.state = self.mode_C
        self.u = 0

        self.x_path = []
        self.y_path = []
        self.theta_path = []
        self.e_path = []

        self.u_path = []
        self.mode_path = []
        self.dR_path = []
        self.ddR_path = []
        self.sat_path = []
        self.second_part_path = []
        self.sgn_path = []

        self.n = 1
        self.v_A = self.disk.disk_center
        self.vA_path = []
        self.p_path = []
        self.d_path = []
        self.r_path = []

    
    def init_pose(self, init_x, init_y, init_theta):
        self.x = init_x
        self.y = init_y
        self.theta = init_theta
        self.e = np.array([
            [np.cos(self.theta)],
            [np.sin(self.theta)]
        ])
        self.x_path = [init_x]
        self.y_path = [init_y]
        self.theta_path = [init_theta]
    
    def update_pose(self, dt):
        self.x += self.linear_velocity * self.e[0][0] * dt
        self.y += self.linear_velocity * self.e[1][0] * dt
        # Store the path
        self.x_path.append(self.x)
        self.y_path.append(self.y)

    
    def lidar_scan(self, obstacles):
        # LiDAR scan
        for i, angle in enumerate(self.lidar.angles_of_rays[0:-1]):
            # Add ray's coridnate in list
            lidar_data_x = self.x + self.lidar.lidar_range * np.cos(self.theta + angle)
            lidar_data_y = self.y + self.lidar.lidar_range * np.sin(self.theta + angle)
            self.lidar.lidar_points[i][0] = lidar_data_x
            self.lidar.lidar_points[i][1] = lidar_data_y

            # Find intersection with obstacles
            for obs in obstacles:
                intersection = vec_ops.find_intersection(np.array([[self.x, self.y], [lidar_data_x, lidar_data_y]]), obs)

                if (intersection is not None) and (np.linalg.norm(intersection-[self.x, self.y]) < np.linalg.norm(self.lidar.lidar_points[i]-[self.x, self.y])):  #!!!!!!!!!!!!!!!!
                    self.lidar.lidar_points[i][0] = intersection[0]
                    self.lidar.lidar_points[i][1] = intersection[1]

            # Calculate distance for ray
            self.lidar.lidar_distances[i] = np.linalg.norm(self.lidar.lidar_points[i] - [self.x, self.y])
        
        # Add noise
        self.lidar.simulate_lidar_measurement(0.02)
        # print(self.lidar.lidar_distances)

        # Filter lidar data
        core = 3
        # self.lidar.lidar_distances = self.lidar.sma(core)
        # print(self.lidar.lidar_distances)
        
        # Update closest params
        self.lidar.closest_distance = np.min(self.lidar.lidar_distances)
        self.lidar.lidar_closest_point = self.lidar.lidar_points[np.argmin(self.lidar.lidar_distances)]            

    def update_disk_center(self):
        self.disk.disk_center = vec_ops.find_vector_with_dir(
            np.array([self.x, self.y]),
            self.lidar.lidar_closest_point,
            self.d + self.disk.R
        )
    
    def update_disk_rays(self):
        target_index = np.argmin(self.lidar.lidar_distances)
        start = target_index - 10
        end = target_index + 10
        interval = np.arange(start, end, 1)
        for i in range(self.lidar.lidar_resolution-1):
            if i in interval:
                self.disk.disk_rays_lengths[i] = self.lidar.lidar_range + 1000
                continue
            self.disk.disk_rays_lengths[i] = np.linalg.norm(self.lidar.lidar_points[i] - self.disk.disk_center)
        
        self.disk.min_disk_length = np.min(self.disk.disk_rays_lengths)
        self.disk.min_arg = np.argmin(self.disk.disk_rays_lengths)
    

    # def switch_global_mode(self):
        


    def switch_mode_in_main(self):
        if self.state_global == self.mode_First:
            if self.lidar.closest_distance < (self.d + 1.2 * self.R_min):
                self.state_global = self.mode_Main
        else:
            if self.state == self.mode_C:
                if self.disk.min_disk_length < np.linalg.norm(self.disk.disk_center-self.lidar.lidar_closest_point):
                    self.state = self.mode_G
                    self.v_A = self.disk.disk_center
            else:
                if not vec_ops.is_point_in_angle(
                    self.v_A,
                    self.lidar.lidar_closest_point,
                    self.lidar.lidar_points[self.disk.min_arg],
                    np.array([self.x, self.y])):
                        self.state = self.mode_C
    
    def calc_u_mode_C(self):
        r = np.array([self.x, self.y])
        p = self.lidar.lidar_closest_point
        dR = self.lidar.closest_distance - self.d
        ddR = self.linear_velocity * (((r-p) / np.linalg.norm(r-p)) @ self.e)[0]
        sat = vec_ops.saturation(dR, -0.1, 0.1)
        second_part = self.n * 0.025 * sat
        sgn = np.sign(ddR + second_part)

        self.d_path.append(float(self.lidar.closest_distance))
        self.r_path.append((round(float(r[0]),2), round(float(r[1]),2)))
        self.p_path.append((round(float(p[0]),2), round(float(p[1]),2)))
        self.vA_path.append(0)
        self.dR_path.append(float(dR))
        self.ddR_path.append(ddR)
        self.sat_path.append(float(sat))
        self.second_part_path.append(float(second_part))
        self.sgn_path.append(sgn)

        print(f'Mode C | dR={round(dR, 4):>7} ddR={round(ddR, 4):>7}', end=' ')
        print(f'u={self.angular_velocity * sgn:>6}')

        return self.angular_velocity * sgn

    def calc_u_mode_G(self):
        r = np.array([self.x, self.y])
        v = self.v_A
        dR = self.turning_radius - np.linalg.norm(r - v)
        ddR = self.linear_velocity * (((v-r) / np.linalg.norm(v-r)) @ self.e)[0]
        sat = vec_ops.saturation(dR, -0.1, 0.1)
        second_part = self.n * 0.025 * sat
        sgn = np.sign(ddR + second_part)

        self.d_path.append(0.0)
        self.r_path.append((round(float(r[0]),2), round(float(r[1]),2)))
        self.p_path.append(0.0)
        self.vA_path.append((round(float(v[0]),2), round(float(v[1]),2)))
        self.dR_path.append(float(dR))
        self.ddR_path.append(ddR)
        self.sat_path.append(float(sat))
        self.second_part_path.append(float(second_part))
        self.sgn_path.append(float(sgn))

        print(f'Mode G | dR={round(dR, 4):>7} ddR={round(ddR, 4):>7} sec_part={round(second_part, 4):>7}', end=' ')
        print(f'u={self.angular_velocity * sgn:>6} v_A={v}')

        return self.angular_velocity * sgn

    def calculate_global_u(self):
        eps = 0.1
        phi = self.goal
        return self.angular_velocity * np.sign(phi - self.theta)


    def update_orientation(self, dt):
        self.u_path.append(self.u)
        self.mode_path.append("C" if self.state == self.mode_C else "G")
        # self.theta -= self.u * dt
        self.theta += self.u * dt
        self.theta_path.append(self.theta)
        self.e = np.array([[np.cos(self.theta)], [np.sin(self.theta)]])
        self.e_path.append(self.e)
    
    def calculate_u(self):
        if self.state_global == self.mode_First:
            self.u = self.calculate_global_u()
        else:
            if self.state == self.mode_C:
                self.u = self.calc_u_mode_C()
            else:
                self.u = self.calc_u_mode_G()