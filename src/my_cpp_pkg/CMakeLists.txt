cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the default installation prefix to /usr/local
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation prefix" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "~/ros2_ws/install/my_robot_interfaces/share/my_robot_interfaces")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)


add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(cpp_node2 src/my_second_node.cpp)
ament_target_dependencies(cpp_node2 rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client2 src/add_two_ints_client2.cpp)
ament_target_dependencies(add_two_ints_client2 rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher my_robot_interfaces rclcpp example_interfaces)

add_executable(number src/number.cpp)
ament_target_dependencies(number rclcpp example_interfaces)

install(TARGETS 
cpp_node
cpp_node2
robot_news_station
smartphone
add_two_ints_server
add_two_ints_client_no_oop
add_two_ints_client2
hardware_status_publisher
number

DESTINATION lib/${PROJECT_NAME}
)

ament_package()
