"""
The functions and class for the logic of the Drone 2D Simulator.
"""

from Utils import OpticalSensor, DistanceSensor, BatterySensor, DronePIDController, directions_dict
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
        self.is_exploring:bool = False
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
        self.is_in_charging_station:bool = False

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

        # ----- Initialize the Optical Sensor -----
        self.optical_sensor = OpticalSensor()

        # ----- Initialize the Drone PID Controller -----
        self.controller = DronePIDController(0.07, 0, 0.04, 4.5)
        self.forward_controller = DronePIDController(0.9,0, 0.015, 4.5)
        self.narrow_controller = DronePIDController(0.03,0, 0.03, 4.5)


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
        x = math.cos(math.radians(direction_calculation)) * self.optical_sensor._currentSpeed
        y = math.sin(math.radians(direction_calculation)) * self.optical_sensor._currentSpeed
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

    def battery_charge(self) -> None:
        """
        Reload the battery of the drone.
        """
        self.battery.fill_battery(amount_to_fill=0.5)
        if self.battery.is_full():
            self.is_in_charging_station = False
            self.is_exploring = True

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
        if not self.is_in_charging_station:
            self.battery.update_battery_level()

        self.fr_dinstance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)
        self.fr_right_distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)
        self.fr_left_distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)
        self.back_distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)
        self.right_distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)
        self.left_distance.distance_update(radius=radius, matrix=matrix, location=location, orientation=orientation)

    def adjust_position_when_near_wall(self) -> None:
        """
        Adjust the position of the drone when it is near a wall.
        """
        self.correction_pid = False
        # ----- If the drone is near the left wall -----
        new_angle = 10
        # ----- Check if the drone is near the right wall -----
        if self.drone_right_tilt:
            new_angle = -10
        self.update_angle(new_angle)

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

    def explore_last_point(self, current_direction):
        """
        After charging, go to the last point before charging to continue exploring.

        Parameters
        ----------
        - `current_direction`: The location of the drone.
        """
        # Get the next point for the drone to move to
        desired_location = self.path_to_explore.pop(0)

        # Determine a point to aim the drone at for better orientation
        desired_direction = self.path_to_explore[2] if len(self.path_to_explore) >= 3 else None

        # If we have reached the end of the path, clear the path and update the exploring flag
        if not self.path_to_explore:
            self.path_to_explore.clear()
            self.is_exploring = False

        # Update the drone's angle to face the determined point if available
        if desired_direction:
            self.adjust_angle_to_direction(current_direction=current_direction, desired_direction=desired_direction)

        else:
            print(f'No available path left to explore.\nPath: {self.path_to_explore}')

        return desired_location


