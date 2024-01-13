#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

# Define a class for the Custom Info Service
class CustomInfoServiceHandler:
    def __init__(self):
       # Initialize instance variables for the average velocity and distance
        self.average_velocity_x = 0
        self.distance = 0

        # Initialize the ROS node with the name 'custom_info_service'
        rospy.init_node('custom_info_service')
        rospy.loginfo("Custom Info Service node initialized")

        # Provide a ROS service named 'info_service', using the custom service type Ave_pos_vel
        rospy.Service("info_service", Ave_pos_vel, self.handle_info_service_request)
        # Subscribe to the '/pos_vel' ROS topic, using the custom message type Vel
        rospy.Subscriber("/pos_vel", Vel, self.calculate_distance_and_average_velocity)

    # Callback function for the '/pos_vel' ROS topic
    def calculate_distance_and_average_velocity(self, vel_message):
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

    # Callback function for the ROS service
    def handle_info_service_request(self, _):
        # Return a response containing the distance and average velocity
        return Ave_pos_velResponse(self.distance, self.average_velocity_x)		      

    # Function to keep the ROS node running
    def run_info_service_node(self):
        rospy.spin()

# Main entry point
if __name__ == "__main__":
    # Instantiate the Custom Info Service class
    info_service_handler = CustomInfoServiceHandler()
    
    # Create a ServiceProxy to invoke the 'info_service' ROS service
    dist_vel_service = rospy.ServiceProxy('info_service', Ave_pos_vel)

    while not rospy.is_shutdown():
       # Invoke the service
        response = dist_vel_service()
        rospy.loginfo(f"Service response:\n {response}")

   # Start the Custom Info Service node
    info_service_handler.run_info_service_node()
