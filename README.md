Research Track1_Assignment2
=================================
Student name: Mohammad Ali Pouryaghoub Shahkhali

Student number: 6063201

This Python script serves as a custom goal handler for a ROS (Robot Operating System) environment. It interacts with the `/odom` topic to publish position and velocity, and it allows users to set or cancel goals through the console. Goals are managed using an action client, and the status of the goals is logged. The script utilizes the `geometry_msgs`, `nav_msgs`, `actionlib`, and custom message types from the `assignment_2_2023` package. The node is initialized as 'custom_goal_handler_node'.

To run the code, ensure the necessary ROS installation and the [RT1_assignment_2](https://github.com/LemmaMoto/RT1_assignment_2.git) repository are available. Also, make sure to grant execution permissions to the Python files within the 'scripts' folder. Finally, execute the script using the following command:

```pythin
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


python```


