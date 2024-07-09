"""
This file represent all the classes and functions that are used in the Drone 2D Simulator.
Such as the Battery sensor, Optical Flow sensor, Distance sensor and Process ID.
"""

import math

directions_dict = {"leftward": 270, "rightward": 90, "forward": 0, "backward": 180, "forward_right": 40, "forward_left": 320}

class DronePIDController:
    """
    Parameters
    --------
    - `kp`: The proportional gain - determines how aggressively the controller reacts to the current error.
    - `ki`: The integral gain - determines how much the controller accumulates the error over time.
    - `kd`: The derivative gain - determines how much the controller reacts to the rate at which the error has been changing.
    - `max_integral`: Limits the integral term to prevent excessive accumulation of past errors.
    """
    def __init__(self, kp, ki, kd, max_integral):
        self.integral: float = 0.0
        self.last_error: float = 0.0
        self.kd = kd
        self.kp = kp
        self.ki = ki
        self.max_integral = max_integral
    
    def update_pid(self, error, dt):
        """
        Proportional-Integral-Derivative control algorithm.
        This algorithm is used to compute a control output that aism to minimize the error between
        a desired setpoint and the current state of the system.

        Parameters
        --------
        - `error`: The difference between the desired setpoint and the current state of the system.
        - `dt`: The time difference between the current and previous iteration.
        """

        self.integral += self.ki * error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        
        derivative = 0.0
        if dt != 0:
            derivative = (error - self.last_error) / dt
        control_output = self.kp * error + self.kd * derivative + self.integral
        
        self.last_error = error
        return control_output
    
    @staticmethod
    def tune_pid(method = 'ziegler-nochols'):
        if method == 'ziegler-nochols':
            return {
                'kp': 0.6,
                'ki': 1.2,
                'kd': 0.075,
                'max_integral': 4.5
            }
        else:
            return {
                'kp': 0.07,
                'ki': 0,
                'kd': 0.04,
                'max_integral': 4.5
            }

class BatterySensor:
    """
    Battery Sensor class that represents the battery sensor of the drone.
    """
    def __init__(self):
        self._battery_level: float = 100
        self._total_time: int = 4800
        self._time_passed = self._total_time

    def is_full(self) -> bool:
        """
        Check if the battery is full.
        """
        return self._battery_level == 100

    def update_battery_level(self) -> None:
        """
        Update the battery level of the drone.
        """
        if self._battery_level > 0:
            self._time_passed -= 1
            temp_time = self._time_passed / self._total_time
            self._battery_level = temp_time * 100

class DistanceSensor:
    """
    Distance Sensor class that represents the distance sensor of the drone.
    """
    def __init__(self, distance:int = 0):
        self._droneGG: int = 120
        self._distance = distance
        self._direction = None

    def update_direction(self, direction: str):
        """
        Update the direction of the drone.

        Parameters
        --------
        `direction`: The direction of the drone.
        """
        self._direction = direction

    def distance_update(self, radius, matrix, location, orientation):
        """
        Update the distance of the drone.

        Parameters
        --------
        `radius`: The radius of the drone.
        `matrix`: The matrix representing the Map.
        `location`: The location of the drone.
        `orientation`: The orientation of the drone.
        """
        angle = orientation + directions_dict[self._direction]
        x = math.cos(math.radians(angle))
        y = math.sin(math.radians(angle))
        dx, dy = location

        if x < 0:
            sensor_x = dx - radius
        else:
            sensor_x = dx + radius
        if y < 0:
            sensor_y = dy - radius
        else:
            sensor_y = dy + radius

        count_distance = 1
        matrix_length = len(matrix)
        matrix_width = len(matrix[0])
        while count_distance <= self._droneGG + 1:
            num_y = int(sensor_y + y * count_distance)
            num_x = int(sensor_x + x * count_distance)
            if num_y < 0 or num_x < 0 or num_x >= matrix_width or num_y >= matrix_length:
                self._distance = count_distance * 2.5
                return
            if matrix[num_y][num_x] == 1:
                self._distance = count_distance * 2.5
                return
            count_distance += 1
        self._distance = self._droneGG * 2.5