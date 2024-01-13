Research Track1_Assignment2
=================================
Student name: Mohammad Ali Pouryaghoub Shahkhali

Student number: 6063201

This Python script serves as a custom goal handler for a ROS (Robot Operating System) environment. It interacts with the `/odom` topic to publish position and velocity, and it allows users to set or cancel goals through the console. Goals are managed using an action client, and the status of the goals is logged. The script utilizes the `geometry_msgs`, `nav_msgs`, `actionlib`, and custom message types from the `assignment_2_2023` package. The node is initialized as 'custom_goal_handler_node'.

To run the code, ensure the necessary ROS installation and the [RT1_assignment_2](https://github.com/LemmaMoto/RT1_assignment_2.git) repository are available. Also, make sure to grant execution permissions to the Python files within the 'scripts' folder. Finally, execute the script using the following command:

```python
#! /usr/bin/env python3

# Importing required libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

# Defining the main class
class CustomGoalHandler:
    def __init__(self):
        # Initialize publisher and action client
        self.velocity_publisher = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.action_client.wait_for_server()
        self.goal_cancelled = True  # Flag indicating whether the current goal has been cancelled

    # Process user input and manage goals
    def process_user_input_and_manage_goals(self):
        while not rospy.is_shutdown():
            # Subscribe to /odom topic and publish position and velocity
            rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
            # Get user command
            user_choice = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            # Get current target position
            current_target_pos_x = rospy.get_param('/des_pos_x')
            current_target_pos_y = rospy.get_param('/des_pos_y')

            # Create a new goal with the current target position
            current_goal = assignment_2_2023.msg.PlanningGoal()
            current_goal.target_pose.pose.position.x = current_target_pos_x
            current_goal.target_pose.pose.position.y = current_target_pos_y
            rospy.loginfo("Current goal: target_x = %f, target_y = %f", current_target_pos_x, current_target_pos_y)

            if user_choice == 'y':
                try:
                    # Get new goal coordinates from user
                    new_goal_x = float(input("Enter the x-coordinate for the new goal: "))
                    new_goal_y = float(input("Enter the y-coordinate for the new goal: "))
                except ValueError:
                    rospy.logwarn("Invalid input. Please enter a valid number.")
                    continue

                # Update target position parameters and the goal
                rospy.set_param('/des_pos_x', new_goal_x)
                rospy.set_param('/des_pos_y', new_goal_y)
                current_goal.target_pose.pose.position.x = new_goal_x
                current_goal.target_pose.pose.position.y = new_goal_y
                
                # Send the new goal to the action server
                self.action_client.send_goal(current_goal)
                self.goal_cancelled = False

            elif user_choice == 'c':
                if not self.goal_cancelled:
                    # Cancel the current goal if there is one
                    self.goal_cancelled = True
                    self.action_client.cancel_goal()
                    rospy.loginfo("Current goal has been cancelled")
                else:
                    rospy.loginfo("No active goal to cancel")
            else:
                rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

            rospy.loginfo("Last received goal: target_x = %f, target_y = %f", current_goal.target_pose.pose.position.x, current_goal.target_pose.pose.position.y)

    # Callback function to publish current position and velocity
    def publish_position_velocity(self, odom_message):
        current_position = odom_message.pose.pose.position
        current_linear_velocity = odom_message.twist.twist.linear
        current_angular_velocity = odom_message.twist.twist.angular

        # Create a new Vel message with the current position and velocity
        current_pos_and_vel = Vel()
        current_pos_and_vel.pos_x = current_position.x
        current_pos_and_vel.pos_y = current_position.y
        current_pos_and_vel.vel_x = current_linear_velocity.x
        current_pos_and_vel.vel_z = current_angular_velocity.z

        # Publish the Vel message
        self.velocity_publisher.publish(current_pos_and_vel)

if __name__ == '__main__':
    # Initialize the node and start handling goal commands
    rospy.init_node('custom_goal_handler_node')
    custom_handler = CustomGoalHandler()
    custom_handler.process_user_input_and_manage_goals()


```

# Simplified Pseudocode for Custom Goal Handling Node A

```bash
# Initialize ROS node and required libraries
Initialize ROS node 'custom_goal_handler_node'
Import required libraries

# Define main class
class CustomGoalHandler:
    # Initialize class variables
    Initialize velocity_publisher, action_client, goal_cancelled

    # Constructor
    def __init__(self):
        # Initialize publisher and action client
        self.velocity_publisher = create_publisher("/pos_vel", Vel, queue_size=1)
        self.action_client = create_action_client('/reaching_goal', PlanningAction)
        Wait for action server
        Initialize goal_cancelled

    # Method to process user input and manage goals
    def process_user_input_and_manage_goals(self):
        Loop while ROS is running:
            # Subscribe to /odom topic and publish position and velocity
            Subscribe to "/odom" and call self.publish_position_velocity
            # Get user command
            user_choice = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            # Get current target position
            current_target_pos_x = get_param('/des_pos_x')
            current_target_pos_y = get_param('/des_pos_y')

            # Create a new goal with the current target position
            current_goal = create_PlanningGoal()
            current_goal.target_pose.pose.position.x = current_target_pos_x
            current_goal.target_pose.pose.position.y = current_target_pos_y
            Log current goal information

            # Handle user input
            if user_choice == 'y':
                # Get new goal coordinates from user
                new_goal_x, new_goal_y = get_user_input_for_new_goal()

                # Update target position parameters and the goal
                Update parameters '/des_pos_x' and '/des_pos_y'
                Update current_goal with new coordinates

                # Send the new goal to the action server
                Send current_goal to action server
                Set goal_cancelled to False

            elif user_choice == 'c':
                # Cancel the current goal if there is one
                Check if goal is not already cancelled
                Cancel current goal
                Log cancellation message
                Set goal_cancelled to True

            else:
                Log warning for invalid command

            Log last received goal information

    # Method to publish current position and velocity
    def publish_position_velocity(self, odom_message):
        Extract current position and velocity from Odometry message
        Create Vel message with the current position and velocity
        Publish the Vel message

# Main program
if __name__ == '__main__':
    Initialize ROS node 'custom_goal_handler_node'
    Create instance of CustomGoalHandler
    Call process_user_input_and_manage_goals method
```

# ROS Last Target Service Node B for Storing and Retrieving Desired Positions

#Initialization: The class CustomLastTargetService initializes with default values for the last desired x and y positions.

Node Initialization: The ROS node is initialized with the name 'custom_last_target_service'. A log message indicates the successful initialization of the node.

Callback Function: The result_callback function serves as the callback for the 'input' service. It creates a response message of type InputResponse and sets its x and y inputs to the last desired positions obtained from the ROS parameter server.

Service Advertisement: The run_service_node method advertises the 'input' service with the name 'input' using the custom service type 'Input'.

Service Handling: The node enters a spin loop, keeping it active to handle incoming service requests. When a request is received, the callback function is invoked to generate a response with the last desired x and y positions.

Node Execution: In the main block, an instance of the CustomLastTargetService class is created, and the run_service_node method is called, initiating the execution of Node B.

In summary, Node B provides a service that responds to requests by supplying the last desired x and y positions, allowing other nodes in the ROS system to retrieve this information as needed.







