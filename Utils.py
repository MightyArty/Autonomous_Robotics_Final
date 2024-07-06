"""
This file represent all the classes and functions that are used in the Drone 2D Simulator.
Such as the Battery sensor, Optical Flow sensor, Distance sensor and Process ID.
"""

class ProcessID:
    def __init__(self, id):
        self.id = id

class BatterySensor:
    """
    Battery Sensor class that represents the battery sensor of the drone.
    """
    def __init__(self, battery_level:int = 100):
        self._battery_level = battery_level
        self._total_time = 4800 # Max time of flight 480 * 10 updates per second
        self._time_passed = self._total_time

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
        self._maxSpeed = 3
        self._currentSpeed = 0
        self._a = 1 # acceleration

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
    def __init__(self, distance):
        self.distance = distance