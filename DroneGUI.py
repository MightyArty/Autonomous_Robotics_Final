"""
The functions and class for the GUI of the Drone 2D Simulator.
"""

import math
import random
import time
import pygame
from PIL import Image
from DroneLogic import DroneLogic
import numpy as np

class DroneGUI:
    def __init__(self, map_filepath:str = None) -> None:
        # ----- Init the Pygame -----
        pygame.init()

        # ----- Set the Frequency as 10 times per second -----
        self.freq:int = 100

        # ----- Set the Map Size -----
        self.map_width:int = 1366
        self.map_height:int = 768

        # ----- Set the Button -----
        self.button = {
            "Return Home": pygame.Rect(1380, 250, 260, 70),
            "Quit Simulation": pygame.Rect(1380, 450, 260, 70)
            }
        
        # ----- Set the Screen -----
        self.screen = pygame.display.set_mode((1656, 768))
        pygame.display.set_caption("Drone Simulator")
        self.already_explored_area = self.create_already_explored_area()

        # ----- Load the Map for the Simulation -----
        if map_filepath:
            self.load_map(map_filepath)
        else:
            raise Exception("No map file provided. Exiting the simulation.")

        # ----- Set the Mouse -----
        self.mouse = pygame.mouse.get_pos()

        # ----- Battery Text -----
        self.battery_text_display = {
            "Drone's battery": "0 %",
        }

        # ----- Set the Colors of the Buttons -----
        self.home_button_color = (80, 80, 80)
        self.quit_button_color = (255, 100, 100)
        self.hover_color = (130, 130, 130)

        # ----- Set the Drone logic -----
        self.drone: DroneLogic = DroneLogic()
        self.drone_location = None
        self.is_game_over:bool = False
        self.found_area = set()
        self.found_area_marked = set()
        self.drone_locations:list = []
        self.sensor_optical_distance:float = 3

        # ----- Pick a starting point on Map for the Drone -----
        while True:
            point_a = random.randint(self.drone.drone_radius, self.map_width - self.drone.drone_radius - 1)
            point_b = random.randint(self.drone.drone_radius, self.map_height - self.drone.drone_radius - 1)
            if not self.close_to_hit(point_a=point_a, point_b=point_b):
                self.drone_location = [point_a, point_b]
                self.helipad_pos = [point_a, point_b]
                break

        # ----- Set the Clock -----
        self.clock = pygame.time.Clock()

        # ----- Set the Flying area -----
        self.flying_area = np.sum(np.array(self.map_matrix) == 0)

        # ----- Set the Drone's Position -----
        self.drone.update_position(self.drone_location)

    def create_already_explored_area(self) -> pygame.Surface:
        """
        Create and return a new explored area surface.
        """
        detected_surface = pygame.Surface((self.map_width, self.map_height))
        detected_surface.set_colorkey((0, 0, 0))  # Set transparent color
        return detected_surface
    
    def reset_already_explored_area(self) -> None:
        """
        Reset the detected area by filling it with the colorkey color.
        """
        self.already_explored_area.fill((0, 0, 0))  # Fill with the transparent color

    def load_map(self, filepath:str) -> None:
        """
        Load the map, set the matrix of the map and set the map colors.

        Parameters
        --------
        `filepath`: The path to the file.
        """
        # Load and resize the image
        image = Image.open(filepath)
        resized_image = image.resize((self.map_width, self.map_height))
        
        rgba_image = resized_image.convert('RGBA')
        pixel_data = rgba_image.load()
        collision_matrix = []
        
        for row_index in range(self.map_height):
            row = []
            for col_index in range(self.map_width):
                if pixel_data[col_index, row_index][:3] <= (50, 50, 50):
                    pixel_data[col_index, row_index] = (128, 128, 128, pixel_data[col_index, row_index][3])
                    row.append(1)
                else:
                    row.append(0)
        
            collision_matrix.append(row)
        
        final_image = Image.new('RGB', (1656, 768), (255, 255, 255))
        final_image.paste(rgba_image, (0, 0))
        
        self.map_img = pygame.image.fromstring(final_image.tobytes(), final_image.size, final_image.mode)
        self.map_matrix = collision_matrix

    def close_to_hit(self, point_a:float, point_b:float) -> bool:
        """
        Check if the drone is close to hitting a wall.

        Parameters
        --------
        `point_a`: The x-coordinate of the drone.
        `point_b`: The y-coordinate of the drone.

        Returns
        --------
        `bool`: True if the drone is close to hitting a wall, False otherwise.
        """
        # ----- Define the range of indices to check -----
        x_min = max(int(point_a - self.drone.drone_radius), 0)
        x_max = min(int(point_a + self.drone.drone_radius), self.map_width - 1)
        y_min = max(int(point_b - self.drone.drone_radius), 0)
        y_max = min(int(point_b + self.drone.drone_radius), self.map_height - 1)

        # ----- Create arrays of indices -----
        x_indices = np.arange(x_min, x_max + 1)
        y_indices = np.arange(y_min, y_max + 1)

        # ----- Generate all combinations of x and y indices -----
        for x in x_indices:
            for y in y_indices:
                if self.map_matrix[y][x] == 1:
                    return True
        return False

    def can_move_without_hitting(self, new_location) -> None:
        """
        Function for checking if the drone can move without hitting a wall.

        Parameters
        --------
        `new_location`: The new location of the drone.
        """
        x, y = new_location
        radius = self.drone.drone_radius
        within_x_bounds = radius <= x < self.map_width - radius
        within_y_bounds = radius <= y < self.map_height - radius
        if within_x_bounds and within_y_bounds:
            if not self.close_to_hit(x, y):
                self.drone_location = new_location
                self.drone.update_position(new_location)
                self.drone_locations.append(new_location[:])
            else:
                self.respawn_on_colison()
        else:
            self.respawn_on_colison()

    def respawn_on_colison(self) -> None:
        """
        In case of colison, respawn the Drone, update the position and all Drone Logic.
        """
        # ----- Update Drone flags and clear all stored path data -----
        self.reset_drone_data(drone=self.drone)
        self.found_area.clear()
        self.found_area_marked.clear()
        self.drone_locations.clear()

        # ----- Generate a correct random position for the Drone -----
        while True:
            new_x = random.randint(self.drone.drone_radius, self.map_width - self.drone.drone_radius - 1)
            new_y = random.randint(self.drone.drone_radius, self.map_height - self.drone.drone_radius - 1)
            if not self.close_to_hit(new_x, new_y):
                self.drone_location = [new_x, new_y]
                self.helipad_pos = [new_x, new_y]
                break
        
        self.reset_already_explored_area()

    @staticmethod
    def reset_drone_data(drone) -> None:
        assert isinstance(drone, DroneLogic)
        drone.want_to_go_home = False
        drone.is_exploring = False
        drone.at_start_point = False
        drone.path_to_explore.clear()
        drone.path_to_start.clear()
        drone.battery._battery_level = 100
        drone.battery._time_passed = 4800
        drone.path.clear()
    
    def draw_button(self, rect, color, text) -> None:
        """
        Function for drawing a button on the screen.
        """
        text_color = (255, 255, 255) # White
        pygame.draw.rect(self.screen, color, rect)
        font = pygame.font.SysFont(None, 24, bold=True)
        text_surf = font.render(text, True, text_color)
        text_rect = text_surf.get_rect(center=rect.center)
        self.screen.blit(text_surf, text_rect)
    
    def background_task(self) -> None:
        """
        Function for updating the background of the screen.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_game_over = True
            if event.type == pygame.MOUSEBUTTONDOWN:
                for k, v in self.button.items():
                    if self.button[k].collidepoint(self.mouse) and k=="Return Home":
                        self.drone.want_to_go_home = True
                    if self.button[k].collidepoint(self.mouse) and k=="Quit Simulation":
                        self.is_game_over = True
    
    def find_closest_point(self, sensor_distance, angle_offset) -> list:
        """
        Function for finding the closest point to the Drone.

        Parameters
        --------
        `sensor_distance`: The distance of the sensor.
        `angle_offset`: The angle offset of the sensor.

        Returns
        --------
        `list`: The closest point to the Drone.
        """
        angle_rad = math.radians((self.drone.drone_orientation + angle_offset) % 360)
        points = []

        for dist in range(1, int(min(sensor_distance, 300) / self.sensor_optical_distance) + 1):
            x = self.drone_location[0] + dist * math.cos(angle_rad)
            y = self.drone_location[1] + dist * math.sin(angle_rad)
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.map_matrix[int(y)][int(x)] == 0:  # Check if the point is in the white area
                    points.append((int(x), int(y)))
        return points
    
    def generate_correct_points(self,x, y) -> set:
        """
        Function for generating the correct points.

        Parameters
        --------
        `x`: The x-coordinate of the point.
        `y`: The y-coordinate of the point.

        Returns
        --------
        `set`: The set of correct points.
        """
        points = set()

        # Generate a grid of points around (0, 0)
        dx, dy = np.meshgrid(np.arange(-2, 3), np.arange(-2, 3))
        
        # Calculate the distance from (0, 0)
        distance = dx**2 + dy**2
        
        # Create a mask for points within the specified radius
        mask = distance <= 4
        
        # Apply the mask and compute new points
        dx, dy = dx[mask], dy[mask]
        
        for offset_x, offset_y in zip(dx, dy):
            new_x, new_y = x + offset_x, y + offset_y
            if 0 <= new_x < self.map_width and 0 <= new_y < self.map_height:
                if self.map_matrix[new_y][new_x] == 0:  # Check if the point is in the white area
                    points.add((new_x, new_y))
                    
        return points
    
    def mark_explored_points(self) -> None:
        """
        Function for marking the explored points on the screen.
        """
        points_to_mark = set()

        # Get detected points from sensors and update points to be painted
        for distance, angle in [(self.drone.left_distance._distance, -90), (self.drone.right_distance._distance, 90)]:
            sensor_points = self.find_closest_point(distance, angle)
            new_detected_points = set(sensor_points) - self.found_area
            for x, y in new_detected_points:
                points_to_mark.update(self.generate_correct_points(x, y))

        self.found_area.update(points_to_mark)
        for index, (x, y) in enumerate(points_to_mark):
            if index % 2 == 0:
                self.already_explored_area.set_at((x, y), (170, 250, 187))

        self.found_area_marked.update(points_to_mark)
        self.screen.blit(self.already_explored_area, (0, 0))

    def Simulate(self) -> None:
        """
        The main function for the Drone Simulator.
        """
        pygame_timer = pygame.time.get_ticks()
        current_time = time.time()

        while not self.is_game_over:
            # ----- The Main loop of the GUI -----
            temp_time = time.time()
            elapsed_time = temp_time - current_time
            current_time = temp_time
            self.mouse = pygame.mouse.get_pos()
            self.background_task()

            # ----- Update sensors 10 times per second -----
            current_pygame_time = pygame.time.get_ticks()
            if current_pygame_time - pygame_timer >= self.freq:
                self.drone.update_sensors(matrix=self.map_matrix, location=self.drone_location, radius=self.drone.drone_radius, orientation=self.drone.drone_orientation)
                pygame_timer = current_pygame_time
            
            # ----- Update Drone location via positioning algorithm -----
            self.drone_location = self.drone.update_position_algo(self.drone_location, elapsed_time, self.map_matrix, self.drone.drone_radius)

            # ----- Check if Drone can move -----
            self.can_move_without_hitting(self.drone_location)

            # ----- Upgrade Map to the GUI screen -----
            self.screen.blit(self.map_img, (0, 0))
            self.mark_explored_points()

            # ----- Draw the Drone -----
            drone_image = pygame.image.load('images/drone.png')
            drone_image = pygame.transform.scale(drone_image, (20,20))
            drone_x, drone_y = self.drone_location
            self.screen.blit(drone_image, (drone_x - drone_image.get_width() // 2, drone_y - drone_image.get_height() // 2))

            # ----- Draw the Helipad -----
            helipad_imgage = pygame.image.load('images/helipad.png')  # replace with the actual file name and path
            helipad_imgage = pygame.transform.scale(helipad_imgage, (30, 39))  # adjust the size of the helipad image as needed
            helipad_x, helipad_y = self.helipad_pos
            self.screen.blit(helipad_imgage, (helipad_x - helipad_imgage.get_width() // 2, helipad_y - helipad_imgage.get_height() // 2))

            # ----- Draw the Battery Text -----
            # self.battery_text_display["Drone's battery"] = f"Drone's battery: {self.drone.battery_sensor.battery_percentage:.1f} %"
            self.battery_text_display["Drone's battery"] = f"Drone's battery: {self.drone.battery._battery_level:.1f} %"
            font_size = 28
            font = pygame.font.SysFont(None, font_size)
            battery_text = self.battery_text_display["Drone's battery"]
            battery_text_surface = font.render(battery_text, True, (0, 0, 0))
            self.screen.blit(battery_text_surface, (1380, self.map_height - 50))

            # ----- Draw the Buttons -----
            for k, v in self.button.items():
                if v.collidepoint(self.mouse):
                    self.draw_button(self.button[k], self.hover_color, k)
                else:
                    if k == "Quit Simulation":
                        self.draw_button(self.button[k], self.quit_button_color, k)
                    else:
                        self.draw_button(self.button[k], self.home_button_color, k)
            pygame.display.update()
            self.clock.tick(60)

        pygame.quit()

