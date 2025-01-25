#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import numpy as np
# from functools import partial

# Callback function to handle incoming data
class Rviz_marker_node:
    def __init__(self):
        rospy.init_node("marker_visualizer", anonymous=False)

        self.marker_array = MarkerArray()
        self.flag=0 # flag to read the rostime since on 1st iteration it can give 0 sometimes.
        self.final_msg_flag=0 # flag to print final msg
        self.time_scale_flag=0
        self.time_scale=2
        self.id_counter=[0,0,0,0] # for managing marker ids
        self.duration_window = 100 # 100 readings per topic
        self.gaps_collection=[[],[],[],[]]
        # Create a publisher to the `visualization_marker` topic
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=20)
        self.time = (rospy.Time.now().secs + rospy.Time.now().nsecs/1e9)

        # Subscribe to the topic you want to read data from
        rospy.Subscriber("gap_prediction/predicted_gaps_cv_model", Float32MultiArray, lambda msg: self.topic_callback(msg, "cv_model_gaps",0,(1,0,0)))
        rospy.Subscriber("gap_prediction/predicted_gaps_ca_model", Float32MultiArray, lambda msg: self.topic_callback(msg, "ca_model_gaps",1,(0,1,0)))
        rospy.Subscriber("gap_prediction/real_gaps", Float32MultiArray, lambda msg: self.topic_callback(msg, "real_gaps",2,(0,0,1)))
        rospy.Subscriber("gap_prediction/lidar_gaps", Float32MultiArray, lambda msg: self.topic_callback(msg, "lidar_gaps",3,(1,1,0)))


    def create_marker(self,msg,namespace,sub_idx,idx,color):
        
        # Create a marker
        marker = Marker()
        
        # Set the frame ID and timestamp
        marker.header = Header()
        marker.header.frame_id = "map"  # Change this to the relevant frame
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and id for the marker
        extra =""
        if sub_idx==0: 
            extra = "/gap_12" # gap between farthest and 2nd farthest
        elif sub_idx==1:
            extra = "/gap_23"
        elif sub_idx==2:
            extra = "/gap_31"

        marker.ns = namespace + extra
        marker.id = self.id_counter[idx]
        # print(marker.ns,marker.id)

        # Set the type of marker (e.g., SPHERE, LINE_STRIP, etc.)
        marker.type = Marker.CUBE  # Change to Marker.LINE_STRIP or others if needed

        # Set the action for the marker (add, modify, delete)
        marker.action = Marker.ADD


        # Set the pose of the marker
        marker.pose.position.x = -5+(msg.data[-1]-self.time)*self.time_scale
        # print(marker.pose.position)# Assume `data` has attributes x, y, z
        marker.pose.position.y = 5-idx*3 -sub_idx
        marker.pose.position.z = msg.data[sub_idx]*0.3/2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = 0.1  # Scale along the x-axis
        marker.scale.y = 0.1  # Scale along the y-axis
        marker.scale.z = msg.data[sub_idx]*0.3# Scale along the z-axis

        # Set the color (RGBA)
        marker.color.r = color[0]*(sub_idx+1)/3
        marker.color.g = color[1]*(sub_idx+1)/3
        marker.color.b = color[2]*(sub_idx+1)/3
        marker.color.a = 1.0  # Transparency (1.0 is fully opaque)

        # Set the lifetime of the marker (0 means forever)
        marker.lifetime = rospy.Duration(0)

        # Publish the marker
        self.marker_array.markers.append(marker)    
        self.marker_pub.publish(self.marker_array)


    def calculate_error(self,real_values, predicted_values):
        # Ensure the input lists have the same dimensions
        if len(real_values) != len(predicted_values) or any(len(r) != len(p) for r, p in zip(real_values, predicted_values)):
            raise ValueError("The dimensions of real and predicted values must match.")
        
        # Transpose the data to separate variables
        real_transposed = np.array(real_values).T
        predicted_transposed = np.array(predicted_values).T
        
        # Calculate RMSE for each variable
        rmses = []
        for real, predicted in zip(real_transposed, predicted_transposed):
            rmse = np.sqrt(np.mean((real - predicted) ** 2))
            rmses.append(rmse)
        
        return rmses[:3]


    def final_message(self):
        print("XXX==============================================================================XX")
        print("Gaps between: 1st and 2nd , 2nd and 3rd , 3rd and 1st ")
        print("RMSE Constant Velocity Model:",self.calculate_error(self.gaps_collection[0],self.gaps_collection[2]))
        print("RMSE Constant Acceleration Model:",self.calculate_error(self.gaps_collection[1],self.gaps_collection[2]))
        print("XXX==============================================================================XX")


    def topic_callback(self,msg,namespace,idx,color):
        if all(x == self.duration_window for x in self.id_counter):
            self.marker_pub.publish(self.marker_array)
            if not(self.final_msg_flag) and idx ==0 :
                self.final_message()
                self.final_msg_flag+=1
            return 

        if not(self.flag):
            self.time = (rospy.Time.now().secs + rospy.Time.now().nsecs/1e9)
            self.flag+=1

        self.gaps_collection[idx].append(msg.data)        
        self.create_marker(msg,namespace,0,idx,color)
        self.create_marker(msg,namespace,1,idx,color)
        self.create_marker(msg,namespace,2,idx,color)
        self.id_counter[idx]+=1

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == "__main__":
    # Initialize the ROS node
    try:
        # Create the node object
        node = Rviz_marker_node()
        # Run the node
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
