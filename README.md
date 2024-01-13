Research Track1_Assignment2
=================================
Student name: Mohammad Ali Pouryaghoub Shahkhali

Student number: 6063201

This Python script serves as a custom goal handler for a ROS (Robot Operating System) environment. It interacts with the `/odom` topic to publish position and velocity, and it allows users to set or cancel goals through the console. Goals are managed using an action client, and the status of the goals is logged. The script utilizes the `geometry_msgs`, `nav_msgs`, `actionlib`, and custom message types from the `assignment_2_2023` package. The node is initialized as 'custom_goal_handler_node'.

To run the code, ensure the necessary ROS installation and the [RT1_assignment_2](https://github.com/LemmaMoto/RT1_assignment_2.git) repository are available. Also, make sure to grant execution permissions to the Python files within the 'scripts' folder. Finally, execute the script using the following command:

```bash
$ rosrun <your_package_name> <your_script_name>.py


