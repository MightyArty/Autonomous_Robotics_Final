"""
This file represent all the classes and functions that are used in the Drone 2D Simulator.
Such as the Battery sensor, Optical Flow sensor, Distance sensor and Process ID.
"""

import math

directions_dict = {"leftward": 270, "rightward": 90, "forward": 0, "backward": 180, "forward_right": 40, "forward_left": 320}

# Proportional-Integral-Derivative
class DronePIDController:
    def __init__(self, kp, ki, kd, max_integral):
        self.integral: float = 0.0
        self.last_error: float = 0.0
        self.first_run: bool = True
        self.kd = kd
        self.kp = kp
        self.ki = ki
        self.max_integral = max_integral
    
    def update(self, error, dt):
        if self.first_run:
            self.first_run = False
            self.last_error = error

        self.integral += self.ki * error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        
        derivative = 0.0
        if dt != 0:
            derivative = (error - self.last_error) / dt
        control_output = self.kp * error + self.kd * derivative + self.integral
        
        self.last_error = error
        return control_output
    
    def increment_kp(self, value):
        self.kp += value

    def increment_ki(self, value):
        self.ki += value

    def increment_kd(self, value):
        self.kd += value

class BatterySensor:
    """
    Battery Sensor class that represents the battery sensor of the drone.
    """
    def __init__(self):
        self._battery_level: float = 100
        self._total_time: int = 4800 # Max time of flight 480 * 10 updates per second
        self._time_passed = self._total_time

    def is_full(self) -> bool:
        """
        Check if the battery is full.
        """
        return self._battery_level == 100

    def update_battery_level(self):
        """
        Update the battery level of the drone.
        """
        if self._battery_level > 0:
            self._time_passed -= 1
            temp_time = self._time_passed / self._total_time
            self._battery_level = temp_time * 100
    
    def reset_battery_level(self):
        """
        Reset the battery level of the drone.
        """
        self._battery_level = 100
        self._time_passed = 4800

    def fill_battery(self, amount_to_fill: int):
        """
        Fill the battery of the drone.
        """
        time_to_fill = (4800 / 100) * amount_to_fill
        self._time_passed += time_to_fill
        if self._time_passed > 4800:
            self._time_passed = 4800
        temp_time = self._time_passed / self._total_time
        self._battery_level = temp_time * 100

class OpticalSensor:
    """
    Optical Sensor class that represents the optical sensor of the drone.
    """
    def __init__(self):
        self._maxSpeed: int = 3
        self._currentSpeed: int = 0
        self._a: float = 1 # acceleration

    def update_speed_a(self):
        """
        Update the speed of the drone.
        """
        if self._currentSpeed < self._maxSpeed:
            self._currentSpeed += self._a
        else:
            self._currentSpeed += 0

    def update_slow_speed(self):
        """
        Update the speed of the drone to slow down.
        """
        if self._currentSpeed > 0:
            self._currentSpeed -= self._a
        else:
            self._currentSpeed -= 0

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