#!/usr/bin/env python3

"""
Node C


This module provides functionality for a custom info service in ROS. It initializes a ROS node called 'custom_info_service' and implements a service that calculates the distance between desired and actual positions, as well as the average velocity based on received data from the '/pos_vel' topic.

.. moduleauthor:: Pouryaghoub Mohammad Ali <6063201@studenti.unige.it>
"""

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

class CustomInfoServiceHandler:
    """
    Class responsible for handling the custom info service.
    """

    def __init__(self):
        """
        Initializes the CustomInfoServiceHandler object.

        Initializes instance variables for the average velocity and distance, setting them to 0.
        Sets up the ROS node with the name 'custom_info_service'.
        Advertises the 'info_service' service and subscribes to the '/pos_vel' topic.

        Args:
            None

        Returns:
            None
        """
        # Initialize instance variables for the average velocity and distance
        self.average_velocity_x = 0
        self.distance = 0

        # Initialize the ROS node with the name 'custom_info_service'
        rospy.init_node('custom_info_service')
        rospy.loginfo("Custom Info Service node initialized")

        # Advertise the 'info_service' service
        rospy.Service("info_service", Ave_pos_vel, self.handle_info_service_request)
        # Subscribe to the '/pos_vel' topic
        rospy.Subscriber("/pos_vel", Vel, self.calculate_distance_and_average_velocity)

    def calculate_distance_and_average_velocity(self, vel_message):
        """
        Calculate the distance between desired and actual positions, and average velocity.

        Extracts desired x and y positions from the ROS parameter server.
        Retrieves the window size for velocity calculation from the ROS parameter server.
        Computes the distance between desired and actual positions.
        Computes the average velocity based on received velocity data.

        Args:
            vel_message (assignment_2_2023.msg.Vel): The Vel message containing position and velocity data.

        Returns:
            None
        """
        # Retrieve the desired x and y positions from the ROS parameter server
        desired_x = rospy.get_param('/des_pos_x')
        desired_y = rospy.get_param('/des_pos_y')

        # Retrieve the window size for the velocity calculation from the ROS parameter server
        velocity_window_size = rospy.get_param('/window_size')

        # Extract the actual x and y positions from the received message
        actual_x = vel_message.pos_x
        actual_y = vel_message.pos_y

        # Compute the distance between the desired and actual positions
        desired_coordinates = [desired_x, desired_y]
        actual_coordinates = [actual_x, actual_y]
        self.distance = math.dist(desired_coordinates, actual_coordinates)

        # Compute the average velocity
        if isinstance(vel_message.vel_x, list):
            velocity_data = vel_message.vel_x[-velocity_window_size:]
        else:
            velocity_data = [vel_message.vel_x]

        self.average_velocity_x = sum(velocity_data) / min(len(velocity_data), velocity_window_size)

    def handle_info_service_request(self, _):
        """
        Handle requests for the info_service.

        Constructs a response containing the distance and average velocity calculated previously, and returns it.

        Args:
            _: Placeholder for the service request.

        Returns:
            assignment_2_2023.srv.Ave_pos_velResponse: A response containing the distance and average velocity.
        """
        # Return a response containing the distance and average velocity
        return Ave_pos_velResponse(self.distance, self.average_velocity_x)

    def run_info_service_node(self):
        """
        Run the Custom Info Service node.

        Starts the Custom Info Service node, allowing it to handle service requests.

        Args:
            None

        Returns:
            None
        """
        rospy.spin()

# Main entry point
if __name__ == "__main__":
    # Instantiate the CustomInfoServiceHandler class
    info_service_handler = CustomInfoServiceHandler()

    # Create a ServiceProxy to invoke the 'info_service' ROS service
    dist_vel_service = rospy.ServiceProxy('info_service', Ave_pos_vel)

    while not rospy.is_shutdown():
        # Invoke the service
        response = dist_vel_service()
        rospy.loginfo(f"Service response:\n {response}")

    # Start the Custom Info Service node
    info_service_handler.run_info_service_node()

