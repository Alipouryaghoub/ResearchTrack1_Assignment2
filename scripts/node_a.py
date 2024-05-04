#!/usr/bin/env python3
"""
Node A

This module provides functionality for setting and canceling goals for a robot, as well as publishing its position and velocity information.


This module initializes ROS components, subscribes to the robot's odometry data, and allows users to set or cancel goals for the robot to reach. It also publishes the robot's position and velocity information.

.. moduleauthor:: Pouryaghoub Mohammad Ali <6063201@studenti.unige.it>
"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus
from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal

class RobotGoalHandler:
    """
    Class responsible for handling robot goals and publishing position and velocity information.
    """

    def __init__(self):
        """
        Initialize the RobotGoalHandler class.
        """
        self.velocity_publisher = None
        self.action_client = None
        self.is_goal_cancelled = True

    def initialize_ros_components(self):
        """
        Initialize ROS components.

        This method initializes the ROS node and the necessary publishers and action clients for communication with the robot.

        Args:
            None

        Returns:
            None
        """
        rospy.init_node('set_robot_target_client')
        self.velocity_publisher = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.action_client.wait_for_server()

    def handle_goal_commands(self):
        """
        Handle user input for setting or canceling goals.

        This method continuously prompts the user for input and responds accordingly by setting new goals or canceling current goals.

        Args:
            None

        Returns:
            None
        """
        rospy.Subscriber("/odom", Odometry, self.publish_robot_position_velocity)
        while not rospy.is_shutdown():
            user_input = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            if user_input == 'y':
                self.set_new_goal()
            elif user_input == 'c':
                self.cancel_current_goal()
            else:
                rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

    def set_new_goal(self):
        """
        Set a new goal for the robot.

        This method prompts the user to input new goal coordinates and sends the goal to the robot.

        Args:
            None

        Returns:
            None
        """
        target_x = rospy.get_param('/des_pos_x')
        target_y = rospy.get_param('/des_pos_y')
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_x, target_y)
        try:
            new_goal_x = float(input("Enter the x-coordinate for the new goal: "))
            new_goal_y = float(input("Enter the y-coordinate for the new goal: "))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number.")
            return
        rospy.set_param('/des_pos_x', new_goal_x)
        rospy.set_param('/des_pos_y', new_goal_y)
        goal.target_pose.pose.position.x = new_goal_x
        goal.target_pose.pose.position.y = new_goal_y
        self.action_client.send_goal(goal)
        self.is_goal_cancelled = False

    def cancel_current_goal(self):
        """
        Cancel the current goal.

        This method cancels the currently active goal, if any.

        Args:
            None

        Returns:
            None
        """
        if not self.is_goal_cancelled:
            self.action_client.cancel_goal()
            rospy.loginfo("Current goal has been cancelled")
            self.is_goal_cancelled = True
        else:
            rospy.loginfo("No active goal to cancel")

    def publish_robot_position_velocity(self, message):
        """
        Publish the robot's position and velocity information.

        This method receives odometry data from the robot and publishes its current position and velocity.

        Args:
            message (Odometry): The odometry message containing position and velocity data.

        Returns:
            None
        """
        current_pos = message.pose.pose.position
        current_vel_linear = message.twist.twist.linear
        current_vel_angular = message.twist.twist.angular
        pos_and_vel = Vel()
        pos_and_vel.pos_x = current_pos.x
        pos_and_vel.pos_y = current_pos.y
        pos_and_vel.vel_x = current_vel_linear.x
        pos_and_vel.vel_z = current_vel_angular.z
        self.velocity_publisher.publish(pos_and_vel)

def main():
    """
    Main function to initialize and run the RobotGoalHandler.
    
    This function creates an instance of RobotGoalHandler, initializes ROS components, and starts handling goal commands.

    Args:
        None

    Returns:
        None
    """
    robot_handler = RobotGoalHandler()
    robot_handler.initialize_ros_components()
    robot_handler.handle_goal_commands()

if __name__ == '__main__':
    main()

