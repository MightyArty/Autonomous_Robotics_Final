"""
The main GUI for the Drone 2D Simulator.
"""
from DroneGUI import DroneGUI
import os
from tkinter import Tk
from tkinter.filedialog import askopenfilename

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_directory = os.path.dirname(script_dir)
    input_filepath = os.path.join(parent_directory, 'maps')

    Tk().withdraw()
    map_filepath = askopenfilename(
        initialdir=input_filepath,
        title="Select Map Image",
        filetypes=[("PNG Files", "*.png")]
    )
    
    if not map_filepath:
        raise Exception("No map file selected. Exiting the simulation.")

    simulation = DroneGUI(map_filepath=map_filepath)
    simulation.Simulate()

if __name__ == "__main__":
    main()