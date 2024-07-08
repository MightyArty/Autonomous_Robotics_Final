"""
The functions and class for the GUI of the Drone 2D Simulator.
"""

import random
import pygame
from PIL import Image
from DroneLogic import DroneLogic
import os
import numpy as np

# ----- Directory Global Path -----
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(script_dir)
input_filepath = os.path.join(parent_directory, 'maps')

class DroneGUI:
    def __init__(self) -> None:
        # ----- Init the Pygame -----
        pygame.init()

        # ----- Set the Map Size -----
        self.map_width:int = 1366
        self.map_height:int = 768

        # ----- Set the Buttons -----
        self.buttons = {
            "Return Home": pygame.Rect(1380, 250, 260, 70),
            "Change Map": pygame.Rect(1380, 385, 260, 70),
            }
        
        # ----- Set the Screen -----
        self.screen_width = self.map_width + 290
        self.screen_height = self.map_height
        self.screen = pygame.display.set_mode((self.map_width + 290, self.screen_height))
        pygame.display.set_caption("Drone Simulator")

        # ----- Set the Map itself -----
        self.map_paths = [os.path.join(input_filepath, f) for f in os.listdir(input_filepath) if f.endswith(('png'))]
        self.current_map_index = 0
        self.load_map(self.map_paths[self.current_map_index])

        # ----- Set the Mouse -----
        self.mouse = pygame.mouse.get_pos()

        # ----- Battery Text -----
        self.battery_text_display = {
            "Drone's battery": "0 %",
        }

        # ----- Set the Colors -----
        self.button_color = (80, 80, 80)
        self.hover_color = (130, 130, 130)

        # ----- Set the Drone logic -----
        self.drone: DroneLogic = DroneLogic()
        self.drone_location = None
        self.is_game_over:bool = False
        self.found_area = set()
        self.found_area_marked = set()
        self.drone_locations:list = []

        # ----- Respawn the Drone on init -----
        while True:
            x = random.randint(self.drone.drone_radius, self.map_width - self.drone.drone_radius - 1)
            y = random.randint(self.drone.drone_radius, self.map_height - self.drone.drone_radius - 1)
            #TODO: implement the check_collision function
            if not self.check_collision(x, y):
                self.drone_location = [x, y]
                self.helipad_pos = [x, y]
                break

        # ----- Set the Clock -----
        self.clock = pygame.time.Clock()

        # ----- Set the Flying area -----
        self.flying_area = np.sum(np.array(self.map_matrix) == 0)

        # ----- Set the Drone's Position -----
        self.drone.update_position(self.drone_location)

    def run_simulator():
        pass