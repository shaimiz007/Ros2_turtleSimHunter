import random

# Read the names from the text file
with open('/home/vboxuser/ros2_ws/src/my_py_pkg/my_py_pkg/names1.txt', 'r') as file:
    names = file.read().strip().split(',')

# Randomly select a name
random_name = random.choice(names).strip()

# Print the randomly selected name
print(random_name)
