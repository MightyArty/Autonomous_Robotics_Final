"""
This file represent all the classes and functions that are used in the Drone 2D Simulator.
Such as the Battery sensor, Optical Flow sensor, Distance sensor and Process ID.
"""

class ProcessID:
    def __init__(self, id):
        self.id = id

class BatterySensor:
    def __init__(self, battery_level):
        self.battery_level = battery_level

class OpticalSensor:
    def __init__(self, flow):
        self.flow = flow

class DistanceSensor:
    def __init__(self, distance):
        self.distance = distance