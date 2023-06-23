from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "shai_number")

    number_publisher_node = (
        Node(
            package="my_cpp_pkg",
            executable="number",
            remappings=[remap_number_topic],
            parameters=[{"number_to_publish":10000} ])
    )
  

    counter_node = Node(
        package="my_py_pkg",
        executable="number_counter_server",
        name="my_counter",
        remappings=[remap_number_topic, ("number_counter", "shai_counter")],
    )

    ld.add_action(number_publisher_node)
    ld.add_action(counter_node)

    return ld
