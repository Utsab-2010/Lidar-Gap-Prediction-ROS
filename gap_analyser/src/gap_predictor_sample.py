#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math

class Gap_Computation_Node:
    def __init__(self):
        rospy.init_node('gap_predictor_node')

        self.state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.model_state_callback)
        self.lidar_sub = rospy.Subscriber('/scan',LaserScan,self.lidar_callback)

        self.pub = rospy.Publisher('predicted_gaps',Float32MultiArray)

        self.real_gap=0
        self.cv_model_gap=0
        self.ca_model_gap=0

        self.cylinder_radius=0.1
        self.pos_list = []

    def model_state_callback(self,msg):
    #getting actual position of the obstacles
        pos = [0,0,0,0,0,0]
        for i, name in enumerate(msg.name):
            if name == "cylinder_1":  # Replace with your model's name
                # Extract position (x, y, z) of the model
                pos[0] = msg.pose[i].position.x
                pos[1] = msg.pose[i].position.y
            if name == "cylinder_2":
                pos[2] = msg.pose[i].position.x
                pos[3] = msg.pose[i].position.y
            if name == "cylinder_3":
                pos[4] = msg.pose[i].position.x
                pos[5] = msg.pose[i].position.y   

        self.real_gap = ((pos[0]-pos[2])**2 + (pos[1]-pos[3])**2)**0.5
        print(f"1st Pos: {pos[0],pos[1]}",f"2st Pos: {pos[2],pos[3]}")

    def lidar_callback(self,msg):
        min_3 = msg.range_max
        min_2 = min_1
        min_1 = min_2
        i_1 = 0
        i_2 = 0
        i_3 = 0
        for i,value in enumerate(msg.ranges):
            if value <min:
                min_2,i_2 = min_1 , i_1
                min_1,i_1 = value,i
            elif value < min_2 and value!=min_1:
                min_2 , i_2 = value , i

        pos_1 = ((min_1+self.cylinder_radius)*math.cos(i_1*3.14/360),(min_1+self.cylinder_radius)*math.sin(i_1*3.14/360))
        pos_2 = ((min_2+self.cylinder_radius)*math.cos(i_2*3.14/360),(min_2+self.cylinder_radius)*math.sin(i_2*3.14/360))
        
        sim_time = rospy.Time.now()

        self.pos_list.append((pos_1,pos_2,sim_time))
        if len(self.pos_list >5):
            self.pos_list.pop(0)
        
    def run(self):
        rospy.spin()

    
if __name__ == '__main__':
    try:
        # Create the node object
        node = Gap_Computation_Node()
        # Run the node
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")