#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math

def move_model(name,x,y):

    # Wait for the /gazebo/set_model_state service to be available
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        # Create a service proxy for setting model states
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Define the new state for the model
        model_state = ModelState()
        model_state.model_name = name  # Replace with your model name
        model_state.pose.position.x = x    # Desired x position
        model_state.pose.position.y = y    # Desired y position
        model_state.pose.position.z = 0.0   # Desired z position
        model_state.pose.orientation.x=0.0
        model_state.pose.orientation.y=0.0
        model_state.pose.orientation.z=0.0
        model_state.pose.orientation.w=0.0

        model_state.reference_frame = "world"  # Frame of reference

        # Call the service to update the model's state
        response = set_model_state(model_state)

        if response.success:
            rospy.loginfo(f"Successfully moved the model '{model_state.model_name}'")
        else:
            rospy.logwarn(f"Failed to move the model '{model_state.model_name}': {response.status_message}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def main_node():
    rospy.init_node('obstacle_movement_node')
    speed=rospy.get_param("obstacle_movement_node/object_speed",0.5)
    print("speed",speed)
    # Wait for the simulation clock to start
    # rospy.wait_for_message('/clock', rospy.Time)
    # print("hi")
    scale=1
    rate = rospy.Rate(50)  # 1 Hz
    while not rospy.is_shutdown():
        time = (rospy.Time.now().secs + rospy.Time.now().nsecs/1e9)*speed  #simulation time in secs
        move_model("cylinder_1",scale*2*math.sin(0.5-time),scale*2*math.cos(time))

        move_model("cylinder_2",scale*math.sin(time-0.5),scale*math.cos(time))

        move_model("cylinder_3",0.0,-0.5)

        move_model("turtlebot3_burger",0.0,0.0)
        rospy.loginfo(f"Current simulation time: {time}")
        rate.sleep()

if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
