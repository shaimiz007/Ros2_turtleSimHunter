#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess


def generate_launch_description():
    ld = LaunchDescription()

    publisher_name="Robot_news_station"
    names=tuple(["glskard","bb8","daneel","jander","c3po"])

    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone"
        )
    
    for i in names:
        robot_news_node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name=publisher_name+"_"+i
             )
        ld.add_action(robot_news_node)
     
      # Refresh rqt_graph
    refresh_rqt_graph_node = Node(
        package="rqt_graph",
        executable="rqt_graph",
        name="rqt_graph_launcher",
        output="screen"
)


    def check_rqt_graph_running():
        cmd = "rosnode list | grep /rqt_graph"
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        process.wait()
        output = process.communicate()[0]
        return len(output.strip()) > 0

    if not check_rqt_graph_running():
    # Launch rqt_graph if not o
        rqt_graph_launcher_node = Node(
            package="rqt_graph",
            executable="rqt_graph",
            name="rqt_graph_launcher",
            output="screen",
            arguments=["--window"]
            )
        ld.add_action(rqt_graph_launcher_node)


    ld.add_action(smartphone_node)
    ld.add_action(refresh_rqt_graph_node)
   
    return ld
