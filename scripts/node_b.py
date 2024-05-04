#!/usr/bin/env python3

"""
Node B


This module provides functionality for a custom last target service in ROS. It initializes a ROS node called 'custom_last_target_service' and implements a service that retrieves the last desired x and y positions from ROS parameters.

.. moduleauthor:: Pouryaghoub Mohammad Ali <6063201@studenti.unige.it>
"""

import rospy
from assignment_2_2023.srv import Input, InputResponse

class CustomLastTargetService:
    """
    Class responsible for handling the custom last target service.
    """

    def __init__(self):
        """
        Initializes the CustomLastTargetService object.

        Initializes class variables for the last desired x and y positions.
        Sets up the ROS node with the name 'custom_last_target_service'.

        Args:
            None

        Returns:
            None
        """
        # Initialize class variables for the last desired x and y positions
        self.last_des_x = 0
        self.last_des_y = 0

        # Initialize the node with the name 'custom_last_target_service'
        rospy.init_node('custom_last_target_service')
        rospy.loginfo("Custom Last Target Service node initialized")

    def result_callback(self, _):
        """
        Callback function for processing the result of the service request.

        This method is called when the service request is received. It retrieves the last desired x and y positions
        from ROS parameters and creates a response message with these values.

        Args:
            _: Placeholder for the result of the service request.

        Returns:
            assignment_2_2023.srv.InputResponse: A response message containing the last desired x and y positions.
        """
        # Create a response message
        response = InputResponse()
        # Set the x and y inputs in the response to the last desired positions
        self.last_des_x = rospy.get_param('/des_pos_x')
        self.last_des_y = rospy.get_param('/des_pos_y')
        response.input_x = self.last_des_x
        response.input_y = self.last_des_y

        # Return the response
        return response

    def run_service_node(self):
        """
        Run the CustomLastTargetService node and advertise the 'input' service.

        Args:
            None

        Returns:
            None
        """
        # Advertise the service with the name 'input' and using the custom service type Input
        service = rospy.Service('input', Input, self.result_callback)

        # Keep the node running to handle service requests
        rospy.spin()

if __name__ == '__main__':
    # Run the Last Target Service node
    last_target_service = CustomLastTargetService()
    last_target_service.run_service_node()

