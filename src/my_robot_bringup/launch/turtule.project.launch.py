from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from time import sleep



def generate_launch_description():
    ld = LaunchDescription()
    
    TurtleSimStartNode = Node(package="turtlesim",
                          executable="turtlesim_node",
                          name="turtlesim_starter", )
    ld.add_action(TurtleSimStartNode)

    TurtleSimStartNode = Node(package="my_py_pkg",
                          executable="turtlesim_starter",)
    TurtleHunter=Node(package="my_py_pkg",
                      executable="hunter_turtle",
                      name="turtle_hunter",
                      )
    
    spawn_nodes = []

    for i in range(1):
        spawn_node = Node(
            package="my_py_pkg",
            executable="spawn",
            name=f"spawn_{i}",
        )
        spawn_nodes.append(spawn_node)

    for i, node in enumerate(spawn_nodes):
        delay = launch.actions.TimerAction(
            period=i * 2.5, actions=[node]  # Delay in seconds for each node
        )
        ld.add_action(delay)


        
    
   # ld.add_action(TurtleHunter)

    return ld

