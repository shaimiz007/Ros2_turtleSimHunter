from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main",
            "smartphone = my_py_pkg.smartphone:main",
            "robot_news_station = my_py_pkg.robot_news_station:main",
            "add_two_ints_server=my_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_no_oop=my_py_pkg.add_two_ints_client_no_oop:main",
            "add_two_ints_client=my_py_pkg.add_two_ints_client:main",
            "number=my_py_pkg.number:main",
            "number_counter_server=my_py_pkg.number_counter_server:main",
            "reset_number_client=my_py_pkg.reset_number_client:main",
            "hw_status_publisher=my_py_pkg.hw_status_publisher:main",
            "battery_client=my_py_pkg.battery_client:main",
            "led_server=my_py_pkg.led_server:main",
            "led_server2=my_py_pkg.led_server2:main",
            "spawn=my_py_pkg.spawn:main",
            "hunter_turtle=my_py_pkg.hunter_turtle:main",
            "hunter_turtle_oop=my_py_pkg.hunter_turtle_oop:main",
            "node_killer=my_py_pkg.node_killer:main"
        ]
    },
)
