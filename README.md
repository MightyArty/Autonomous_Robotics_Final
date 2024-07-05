# Final Project in Autonomous Robotics course

## Authors: Tom Shabalin, Dor Harizi and Shai Moshe

## Overview
The primary objective of this project is to develop an effective solution for a small drone to navigate indoors without collisions. This project involves a fully autonomous 2D drone simulator designed to be as realistic as possible. It incorporates LIDAR sensors, a gyroscope, an optical flow sensor, and a speed sensor, with each sensor's readings including a slight amount of noise to enhance realism.

## What was added from the previous implementation
The previous implementation ([Drone Simulator](https://github.com/MightyArty/DroneSimulator)) was implemented in Java, and was not so accurate.\
The drone was hitting the walls very often, going into a loop while not discovering any new areas, and did not have the ability to return Home (starting point).
### The improvements
1. `Return Home ability`: when the drone battery sensor hits 50%, it return to the starting point while using the shortest path possible. In addition, we have implemented a simple Button, that by pressing it the Drone can also return to the starting point.
2. `Avoiding obstacles`: the drone obstacle avoidance is also improved a lot, while adjusting the rotation angle when it's near a wall or an obstacle.
3. `New path finding`: the drone main aim now is to explore new areas, and try to not go back to already explored ares.
4. `Improved GUI`: the GUI is also improved a lot, making it more functional, more understandable and more user friendly.

The GUI was implemented using Pygame library.

## How to run
```bash
# Clone the repository
$ git clone "https://github.com/MightyArty/Autonomous_Robotics_Final"
# Enter the folder
$ cd Autonomous_Robotics_Final
# Install the required libraries
$ Run "pip install -r requirements.txt"
```