import numpy as np
import vector_operations as vec_ops

class LiDAR:
    def __init__(self, lidar_range, lidar_resolution) -> None:
        self.lidar_range = lidar_range            # Maximum range of LiDAR
        self.lidar_resolution = lidar_resolution  # Number of rays in 360 degrees
        self.lidar_points = np.zeros(             # Coordinates of all rays
            (lidar_resolution-1, 2),
            dtype=np.float32
        )
        self.lidar_distances = np.zeros(          # Value of all rays' distances
            lidar_resolution-1,
            dtype=np.float32
        )
        self.angles_of_rays = np.linspace(0, 2 * np.pi, lidar_resolution)
        self.closest_distance = 0
        self.lidar_closest_point = np.array([0, 0])


class TurningDisk:
    def __init__(self, R, resolution):
        self.R = R
        self.disk_center = np.array([0.0, 0.0])
        self.disk_rays_lengths = np.zeros( 
            resolution-1,
            dtype=np.float32
        )
        self.min_disk_length = 0
        self.min_arg = 0

class DubinsCar:

    mode_C = 0
    mode_G = 1

    def __init__(self, linear_velocity, angular_velocity, turning_radius, d) -> None:
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.R_min = linear_velocity / angular_velocity
        self.turning_radius = turning_radius
        self.d = d

        self.x = 0
        self.y = 0
        self.theta = 0
        self.e = np.array([
            [np.cos(self.theta)],
            [np.sin(self.theta)]
        ])
        
        self.lidar = LiDAR(20, 360)
        self.disk = TurningDisk(turning_radius, 360)
        
        self.state = self.mode_C
        self.u = 0

        self.x_path = []
        self.y_path = []
        self.theta_path = []
        self.u = []
    
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
                # Rewrite point if it has intersection with obstacle
                if intersection is not None:
                    self.lidar.lidar_points[i][0] = intersection[0]
                    self.lidar.lidar_points[i][1] = intersection[1]

            # Calculate distance for ray
            self.lidar.lidar_distances[i] = np.linalg.norm(self.lidar.lidar_points[i] - [self.x, self.y])
            # Update closest params
            self.lidar.closest_distance = np.min(self.lidar.lidar_distances)
            self.lidar.lidar_closest_point = self.lidar.lidar_points[np.argmin(self.lidar.lidar_distances)]


    def update_disk_center(self):
        self.disk.disk_center = vec_ops.find_vector_with_dir(
            np.array([self.x, self.y]),
            self.lidar.lidar_closest_point,
            self.d + self.disk.R
        )
        print(self.disk.disk_center)
    
    def update_disk_rays(self):
        # print(self.disk.R, self.d)
        target_index = np.argmin(self.lidar.lidar_distances)
        start = target_index - 10
        end = target_index + 10
        interval = np.arange(start, end, 1)
        for i in range(self.lidar.lidar_resolution-1):
            if i in interval:
                self.disk.disk_rays_lengths[i] = self.lidar.lidar_range + 10
                continue
            self.disk.disk_rays_lengths[i] = np.linalg.norm(self.lidar.lidar_points[i] - self.disk.disk_center)
            # print(self.disk.disk_rays_lengths[i])
        
        self.disk.min_disk_length = np.min(self.disk.disk_rays_lengths)
        self.disk.min_arg = np.argmin(self.disk.disk_rays_lengths)
