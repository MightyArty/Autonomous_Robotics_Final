"""
The functions and class for the logic of the Drone 2D Simulator.
"""
import time

from Utils import DistanceSensor, BatterySensor, DronePIDController, directions_dict
import math

class DroneLogic:
    def __init__(self) -> None:
        # ----- Drone Position -----
        self.drone_right_tilt:bool = True
        self.start_position:tuple = (0,0)
        self.from_wall:int = 25

        # ----- Drone Orientation -----
        self.drone_orientation:int = 0

        # ----- Drone Path parameters -----
        self.path_to_start:list = []
        self.path_to_explore:list = []
        self.path = []

        # ----- Donre Home flag -----
        self.want_to_go_home:bool = False

        # ----- Correction Flag -----
        self.correction_pid:bool = True

        # ----- Drone Relaxation Mode -----
        self.relaxation:bool = False
        self.relax_time:float = 0.0

        # ----- Drone Crash Flag -----
        self.about_to_crash:bool = False

        # ----- Charging if on start point flag -----
        self.at_start_point:bool = False

        # ----- Initialize the sensors -----
        self.fr_distance:DistanceSensor = DistanceSensor()
        self.fr_right_distance:DistanceSensor = DistanceSensor()
        self.fr_left_distance:DistanceSensor = DistanceSensor()
        self.back_distance:DistanceSensor = DistanceSensor()
        self.right_distance:DistanceSensor = DistanceSensor()
        self.left_distance:DistanceSensor = DistanceSensor()
        self.initialize_directions()

        # ----- Initialize the Battery -----
        self.battery = BatterySensor()

        # ----- Drone Raduis -----
        self.drone_radius = int(15 / 2.5)

        # ----- Initialize the Drone PID Controller -----
        self.controller = DronePIDController(kp=0.1, ki=0, kd=0.04, max_integral=4.5)
        self.forward_controller = DronePIDController(kp=0.9, ki=0, kd=0.015, max_integral=4.5)
        self.narrow_controller = DronePIDController(kp=0.03, ki=0, kd=0.03, max_integral=4.5)

    def initialize_directions(self) -> None:
        """
        Initialize the directions of the drone.
        """
        self.fr_distance.update_direction("forward")
        self.fr_right_distance.update_direction("forward_right")
        self.fr_left_distance.update_direction("forward_left")
        self.back_distance.update_direction("backward")
        self.right_distance.update_direction("rightward")
        self.left_distance.update_direction("leftward")

    def move_drone(self, drone_position, desired_direction) -> list:
        """
        Function that moves the drone.

        Parameters
        ----------
        - `drone_position`: The current position of the drone.
        - `desired_direction`: The desired direction of the drone.

        Returns
        -------
        - `drone_position`: The new position of the drone.
        """
        direction_calculation = self.drone_orientation + directions_dict[desired_direction]
        x = math.cos(math.radians(direction_calculation)) * 1
        y = math.sin(math.radians(direction_calculation)) * 1
        return [drone_position[0] + x, drone_position[1] + y]

    def update_angle(self, desired_angle) -> None:
        """
        Update the angle of the drone.

        Parameters
        ----------
        - `desired_angle`: The desired angle of the drone.
        """
        current_orientation = self.drone_orientation
        new_orientation = current_orientation + desired_angle
        # ----- Ensure the angle stays within the 0-359 degree range -----
        self.drone_orientation = new_orientation % 360

    def update_position(self, desired_position) -> None:
        """
        Update the position of the drone.

        Parameters
        ----------
        - `desired_position`: The desired position of the drone.
        """
        if not self.path:
            self.start_position = desired_position
            self.path = [desired_position]
        else:
            if not self.want_to_go_home:
                self.path.append(desired_position)

    def update_sensors(self, matrix, location, radius, orientation) -> None:
        """
        Update all sensors of the drone.

        Parameters
        ----------
        - `matrix`: The matrix representing the Map.
        - `location`: The location of the drone.
        - `radius`: The radius of the drone.
        - `orientation`: The orientation of the drone.
        """
        if not self.at_start_point:
            self.battery.update_battery_level()

        distance_attributes = [
            self.fr_distance,
            self.fr_right_distance,
            self.fr_left_distance,
            self.back_distance,
            self.right_distance,
            self.left_distance
        ]

        # Update each distance attribute
        for distance in distance_attributes:
            distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)

    def toggle_correction_mode(self):
        """
        Switch the drone's correction mode and adjust its angle away from the current wall.

        This method toggles the `correction_pid` flag and adjusts the drone's angle to look
        10 degrees away from the current wall direction.

        Notes
        -----
        - This method assumes `self.drone_right_tilt` determines the current wall direction.
        - `updateDroneAngle` method is assumed to handle angle adjustments.
        """
        self.correction_pid = False
        angle_delta = -10 if self.drone_right_tilt else 10
        self.update_angle(angle_delta)

    def is_in_danger(self) -> bool:
        """
        Check if the drone is in danger, i.e., about to hit a wall.

        Returns
        -------
        - `bool`: True if the drone is in danger, False otherwise.
        """
        distances = [
            self.fr_distance._distance < 22.5,
            self.right_distance._distance < 7.5,
            self.left_distance._distance < 7.5,
            self.fr_right_distance._distance < 7.5,
            self.fr_left_distance._distance < 7.5
        ]
        return any(distances)
    
    def adjust_angle_to_direction(self, current_direction, desired_direction) -> None:
        """
        Adjust the angle of the drone to face the desired direction.

        Parameters
        ----------
        - `current_direction`: The current direction of the drone.
        - `desired_direction`: The desired direction of the drone.
        """
        # Unpack Current and Desired Position
        curr_x, curr_y = current_direction
        desired_x, desired_y = desired_direction

        # Calculate the change in x and y coordinates
        x = desired_x - curr_x
        y = desired_y - curr_y

        # Calculate the angle needed to face the desired direction
        angle = math.degrees(math.atan2(y, x))

        # Update the drone orientation to the desired angle
        self.drone_orientation = angle

    def navigate_last_position(self, drone_position, delta_time) -> list:
        """
        Navigate until 50%.

        Parameters
        ----------
        - drone_position: The current position of the drone.
        - delta_time: The time difference used for PID calculations.
        """
        proximity_status = self.check_proximity_status()

        if not self.correction_pid:
            if self.drone_right_tilt:
                if proximity_status['can_navigate_freely']:
                    return self.move_drone(drone_position=drone_position, desired_direction="forward")

                if proximity_status['near_left_wall']:
                    self.drone_right_tilt = False
                self.correction_pid = True
            else:
                if proximity_status['can_navigate_freely']:
                    return self.move_drone(drone_position=drone_position, desired_direction="forward")

                if proximity_status['near_right_wall']:
                    self.drone_right_tilt = True
                self.correction_pid = True

        overall_correction = self.calculate_pid_adjustments(delta_time)
        self.update_angle(desired_angle=overall_correction)
        return self.move_drone(drone_position=drone_position, desired_direction="forward")

    def check_proximity_status(self) -> dict:
        """
        Check the proximity status of the drone to determine navigation decisions.

        Returns
        -------
        - dict: A dictionary with boolean values indicating proximity status.
            - 'can_navigate_freely': True if the drone should navigate without PID correction.
            - 'near_left_wall': True if the drone is close to the left wall.
            - 'near_right_wall': True if the drone is close to the right wall.
        """
        distances = {
            'front': self.fr_distance._distance,
            'left': self.left_distance._distance,
            'right': self.right_distance._distance,
            'front_left': self.fr_left_distance._distance,
            'front_right': self.fr_right_distance._distance
        }

        threshold = 22  # Threshold distance to consider the drone safe from obstacles

        can_navigate_freely = all(distance > threshold for distance in distances.values())
        near_left_wall = any(distances[side] <= threshold for side in ['front', 'left', 'front_left'])
        near_right_wall = any(distances[side] <= threshold for side in ['front', 'right', 'front_right'])

        return {
            'can_navigate_freely': can_navigate_freely,
            'near_left_wall': near_left_wall,
            'near_right_wall': near_right_wall
        }

    def calculate_pid_adjustments(self, delta_time):
        """
        Calculate the PID corrections for navigating the drone.

        Parameters
        ----------
        - delta_time: The time difference used for PID calculations.

        Returns
        -------
        - float: The overall correction value, limited to a maximum range.
        """
        wall_error = self.calculate_wall_proximity_error()
        forward_error = self.calculate_forward_correction()
        narrow_path_error = self.calculate_narrow_path_adjustment()

        wall_correction = self.controller.update_pid(wall_error, delta_time)
        forward_correction = self.forward_controller.update_pid(forward_error, delta_time)
        narrow_path_correction = self.narrow_controller.update_pid(narrow_path_error, delta_time)

        overall_correction = wall_correction + forward_correction + narrow_path_correction
        max_correction = 10  # Limit the overall correction to a maximum value to avoid excessive adjustments
        return max(-max_correction, min(overall_correction, max_correction))

    def calculate_wall_proximity_error(self):
        """
        Calculate the distance error from the wall for PID correction based on the drone's tilt direction.

        Returns
        -------
        - float: The calculated error based on the distance from the wall.
        """
        # The factor 0.62 is an empirically determined coefficient that scales the distance error
        direction_multiplier = 0.62 if self.drone_right_tilt else -0.62
        distance = self.right_distance._distance if self.drone_right_tilt else self.left_distance._distance
        return direction_multiplier * (distance - self.from_wall)

    def calculate_forward_correction(self):
        """
        Calculate the correction needed based on the forward distance to avoid obstacles.

        Returns
        -------
        - float: The correction value for forward distance, considering the drone's current tilt direction.
        """
        front_danger_distance = 60  # Distance threshold for front obstacles
        forward_distance_error = max(0, front_danger_distance - self.fr_distance._distance)
        turning_direction = -2 if self.drone_right_tilt else 2  # Factor determining the turning direction based on tilt
        return forward_distance_error * turning_direction

    def calculate_narrow_path_adjustment(self):
        """
        Calculate the correction needed when navigating through narrow paths.

        Returns
        -------
        - float: The correction value for navigating narrow paths.
        """
        narrow_path_error = 0
        if self.drone_right_tilt and self.left_distance._distance < self.right_distance._distance:
            narrow_path_error = self.right_distance._distance - self.left_distance._distance
        elif not self.drone_right_tilt and self.left_distance._distance > self.right_distance._distance:
            narrow_path_error = self.right_distance._distance - self.left_distance._distance

        return narrow_path_error  # Return the narrow path error for PID correction

    def update_position_algo(self, drone_position, dt, map_matrix, drone_radius):
        """
        Update the drone's position algorithm based on its current state.

        Parameters
        ----------
        - drone_position: Current position of the drone (tuple of x, y coordinates).
        - dt: Time difference used for calculations.
        - map_matrix: 2D list representing the map with obstacles marked as 1.
        - drone_radius: Radius of the drone for pathfinding considerations.

        Returns
        ----------
        - Tuple: New position for the drone to move towards.
        """
        if self.battery._battery_level <= 50 or self.want_to_go_home:
            return self.return_to_base(map_matrix, drone_position, drone_radius)
        elif self.at_start_point:
            self.battery._battery_level = 0
        else:
            if not self.about_to_crash and not self.is_in_danger():
                return self.fly_near_wall(drone_position, dt)
            else:
                if self.about_to_crash:
                    if not self.is_in_danger():
                        self.about_to_crash = False
                    return self.navigate_last_position(drone_position, dt)
                else:
                    self.about_to_crash = True
        return drone_position

    def fly_near_wall(self, drone_pos, dt) -> list:
        """
        Perform wall following behavior for the drone.

        Parameters
        ----------
        - drone_pos: Current position of the drone (tuple of x, y coordinates).
        - dt: Time difference used for calculations.

        Returns
        -------
        - Tuple: New position for the drone to move towards.
        """
        new_pos = self.navigate_last_position(drone_pos, dt)
        if time.time() - self.relax_time >= 2:
            self.relaxation = False
        correction_flag = any(math.sqrt((new_pos[0] - point[0]) ** 2 + (new_pos[1] - point[1]) ** 2) <= 0.5 for point in self.path)
        if not self.relaxation and correction_flag:
            self.toggle_correction_mode()
            self.relaxation = True
            self.relax_time = time.time()
        return new_pos

    def check_clear_path(self, start, end, grid, drone_width):
        """
        Determine if the path between two points is clear, considering the drone's width.

        Parameters
        ----------
        - `start`: The starting coordinates (x, y) of the path.
        - `end`: The ending coordinates (x, y) of the path.
        - `grid`: The grid representing the map, where obstacles are marked.
        - `drone_width`: The width of the drone.

        Returns
        -------
        - `bool`: True if the path is clear, False otherwise.
        """
        # Double the drone width for calculations
        adjusted_width = drone_width * 2

        # Primary path check
        if not self.is_primary_path_clear(start, end, grid):
            return False

        # Right offset path check
        if not self.is_offset_path_clear(start, end, grid, adjusted_width, right=True):
            return False

        # Left offset path check
        if not self.is_offset_path_clear(start, end, grid, adjusted_width, right=False):
            return False

        return True

    def is_primary_path_clear(self, start, end, grid):
        """
        Check if the primary path between two points is clear.
        """
        primary_path = self.generate_path(start, end)
        return self.is_valid_path(primary_path, grid)

    def is_offset_path_clear(self, start, end, grid, offset_distance, right=True):
        """
        Check if the offset path (either left or right) is clear.
        """
        offset_start, offset_end = self.calculate_offset_points(start, end, offset_distance, right)
        offset_path = self.generate_path(offset_start, offset_end)
        return self.is_valid_path(offset_path, grid)

    def generate_path(self, start, end):
        """
        Generate a path between two points using the DDA algorithm.
        """
        return self.DDA(int(start[0]), int(start[1]), int(end[0]), int(end[1]))

    def calculate_offset_points(self, start, end, offset_distance, right=True):
        """
        Calculate the offset points for the path using vector mathematics.

        Parameters
        ----------
        - `start`: The starting coordinates (x, y) of the path.
        - `end`: The ending coordinates (x, y) of the path.
        - `offset_distance`: The distance to offset the path.
        - `right`: Flag to determine if the offset is to the right or left.
        """
        x = end[0] - start[0]
        y = end[1] - start[1]
        sqrt_distance = math.sqrt(x ** 2 + y ** 2)
        
        if sqrt_distance == 0:
            return start, end  # Avoid division by zero if start and end points are the same

        # Calculate the unit perpendicular vector
        unit_perpendicular = (-y / sqrt_distance, x / sqrt_distance)
        
        if not right:
            unit_perpendicular = (y / sqrt_distance, -x / sqrt_distance)

        offset_vector = (unit_perpendicular[0] * offset_distance, unit_perpendicular[1] * offset_distance)

        offset_start = (start[0] + offset_vector[0], start[1] + offset_vector[1])
        offset_end = (end[0] + offset_vector[0], end[1] + offset_vector[1])

        return offset_start, offset_end

    @staticmethod
    def DDA(x0, y0, x1, y1):
        """
        Perform Digital Differential Analyzer (DDA) algorithm for line drawing between two points.

        Parameters
        ----------
        - `x0`, `y0`: Coordinates of the starting point.
        - `x1`, `y1`: Coordinates of the ending point.

        Returns
        -------
        - `list`: List of points (tuples of x, y) representing the line between (x0, y0) and (x1, y1).
        """
        points = []
        dx = x1 - x0
        dy = y1 - y0
        steps = int(max(abs(dx), abs(dy)))

        if steps == 0:
            return [(x0, y0)]

        x_increment = dx / steps
        y_increment = dy / steps

        x, y = x0, y0
        for _ in range(steps + 1):
            points.append((round(x), round(y)))
            x += x_increment
            y += y_increment

        return points

    @staticmethod
    def is_valid_path(path: list, map_matrix: list) -> bool:
        """
        Check if the given path is clear of obstacles in the map matrix.

        Parameters
        ----------
        - path: List of tuples representing the coordinates of the path.
        - map_matrix: 2D list representing the map with obstacles marked as 1.

        Returns
        ----------
        - bool: True if the path is clear (does not intersect with obstacles), False otherwise.
        """
        rows, cols = len(map_matrix), len(map_matrix[0])
        return all(
            0 <= x < cols and 0 <= y < rows and map_matrix[y][x] == 0
            for x, y in path
        )

    def return_to_base(self, map_matrix, drone_position, drone_radius):
        """
        Navigate the drone back to its starting position or home base.

        Parameters
        ----------
        - `map_matrix`: 2D list representing the map with obstacles marked as 1.
        - `drone_position`: Current position of the drone (tuple of x, y coordinates).
        - `drone_radius`: Radius of the drone for pathfinding considerations.

        Returns
        -------
        - Tuple: Next position for the drone to move towards.
        """
        allowed_error = 10

        if self.is_close_to_start(drone_position, allowed_error):
            self.want_to_go_home = False
            self.at_start_point = True
            self.path_to_start.clear()
            self.path_to_explore.insert(0, drone_position)
            return drone_position

        if self.path_to_start:
            return self.move_along_path_to_start()

        trail_points_within_radius = self.find_points_within_radius(drone_position, map_matrix, drone_radius)

        if not trail_points_within_radius:
            print("No points on trail have been found")
            self.want_to_go_home = False
            return drone_position

        closest_point = self.find_closest_point_to_start(trail_points_within_radius)

        path = self.calculate_path(drone_position, closest_point[1])
        self.path_to_start.extend(path)

        next_position = self.path_to_start.pop(0)
        self.adjust_angle_to_direction(drone_position, path[-1])

        return next_position

    def is_close_to_start(self, drone_position, allowed_error):
        """
        Check if the drone is close to the starting position within the allowed error.

        Parameters
        ----------
        - `drone_position`: Current position of the drone (tuple of x, y coordinates).
        - `allowed_error`: Maximum allowed deviation from the starting position.

        Returns
        -------
        - `bool`: True if the drone is within the allowed error distance from the starting position, False otherwise.
        """
        start_position = self.path[0]

        return (abs(drone_position[0] - start_position[0]) <= allowed_error and
                abs(drone_position[1] - start_position[1]) <= allowed_error)

    def move_along_path_to_start(self):
        """
        Move the drone along the path towards the starting position.

        Returns
        -------
        - Tuple: Next position for the drone to move towards.
        """
        next_position = self.path_to_start.pop(0)
        self.path_to_explore.insert(0, next_position)
        return next_position

    def find_points_within_radius(self, current_position, map_matrix, drone_radius):
        """
        Find points within a given radius from the current position that have a clear line of sight.

        Parameters
        ----------
        - `current_position`: Current position of the drone (tuple of x, y coordinates).
        - `map_matrix`: 2D list representing the map with obstacles marked as 1.
        - `drone_radius`: Radius of the drone for pathfinding considerations.

        Returns
        -------
        - List: List of tuples containing indices and points within radius and clear line of sight.
        """
        return [(idx, point) for idx, point in enumerate(self.path)
                if self.is_within_radius(current_position, point) and
                self.check_clear_path(current_position, point, map_matrix, drone_radius)]

    def is_within_radius(self, current_position, point) -> bool:
        """
        Check if a given point is within the drone's radius from the current position.

        Parameters
        ----------
        - `current_position`: Current position of the drone (tuple of x, y coordinates).
        - `point`: Point to check (tuple of x, y coordinates).

        Returns
        -------
        - `bool`: True if the point is within the drone's radius, False otherwise.
        """
        distance = math.sqrt((current_position[0] - point[0]) ** 2 +
                             (current_position[1] - point[1]) ** 2)
        return distance <= self.left_distance._droneGG * 1.5

    def find_closest_point_to_start(self, points):
        """
        Find the closest point to the starting position.

        Parameters
        ----------
        - `points`: List of tuples containing indices and points.

        Returns
        -------
        - Tuple: Closest point to the starting position.
        """
        def get_key(point):
            return abs(point[0])
        return min(points, key=get_key)

    def calculate_path(self, start_point, end_point) -> list:
        """
        Calculate a path from start point to the end point using the DDA algorithm.

        Parameters
        --------
        - `start_point` (tuple): The starting position of the drone (x, y).
        - `end_point` (tuple): The desired position to reach (x, y).

        Returns
        --------
        -List[tuple]: List of points from current_pos to desired_pos.
        """
        x1, y1 = start_point
        x2, y2 = end_point

        path = self.DDA(int(x1), int(y1), int(x2), int(y2))
        return path