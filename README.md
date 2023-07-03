# ROS2 TurtleSim Hunter Project

This is a project that utilizes ROS2 and TurtleSim to create a turtle hunter scenario. Here are the main features of the project:

1. Complete Project in TurtleSim/Ros2: The project is developed using the TurtleSim simulator in the ROS2 framework.

2. Spawners and Main Turtle ('turtle1'): The project involves creating multiple spawners in the TurtleSim environment. The main turtle, 'turtle1', is responsible for catching and killing these spawners.

3. Random Polynomial Route: The main turtle, 'turtle1', follows a random polynomial route. The route is generated randomly and guides the turtle's movement in the TurtleSim environment.

4. Searching for the Closest Spawner and Teleporting: 'turtle1' continuously searches for the closest spawner among the available ones. Once identified, it teleports to the location of the closest spawner using the generated random polynomial route.

5. Animal Class Instances for Spawners: Each spawner is associated with an instance of the Animal class. This class contains relevant information about the spawner, such as its name, position, and orientation.

6. Launch File for Simulation: The entire simulation is launched using a launch file. This launch file coordinates the execution of different nodes and sets up the TurtleSim environment for the turtle hunting scenario.

This project combines the functionality of spawning, hunting, and teleporting turtles in the TurtleSim environment using ROS2.
