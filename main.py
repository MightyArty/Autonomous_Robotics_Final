"""
The main GUI for the Drone 2D Simulator.
"""
from DroneSimulation import DroneSimulator

def main():
    simulation = DroneSimulator()
    simulation.run_simulator()

if __name__ == "__main__":
    main()