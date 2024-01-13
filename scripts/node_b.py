#!/usr/bin/env python3

import rospy
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Input, InputResponse

# Define a class for the Last Target Service
class CustomLastTargetService:
    def __init__(self):
        # Initialize class variables for the last desired x and y positions
        self.last_des_x = 0
        self.last_des_y = 0

        # Initialize the node with the name 'custom_last_target_service'
        rospy.init_node('custom_last_target_service')
        rospy.loginfo("Custom Last Target Service node initialized")

    # Callback function for the service
    def result_callback(self, _):
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
        # Advertise the service with the name 'input' and using the custom service type Input
        service = rospy.Service('input', Input, self.result_callback)

        # Keep the node running to handle service requests
        rospy.spin()

if __name__ == '__main__':
    # Run the Last Target Service node
    last_target_service = CustomLastTargetService()
    last_target_service.run_service_node()