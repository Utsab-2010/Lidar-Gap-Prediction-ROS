#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
# from dbscan_clustering import DBS_estimation
from kmeans_clustering import Lidar_Pos
from gap_finder import Lidar_gaps
from prediction_models import velocity_model,acceleration_model

def get_gap(pos_1,pos_2):
    return ((pos_1[0] - pos_2[0])**2+(pos_1[1]-pos_2[1])**2)**0.5



class Gap_Computation_Node:
    def __init__(self):
        rospy.init_node('gap_predictor_node')

        # Some useful variables
        self.real_gaps=None
        self.cv_model_gaps=None
        self.ca_model_gaps=None
        self.lidar_gaps=None
        
        self.cylinder_radius=0.1
        self.pos_list = []
        self.edge_point_list=[]
        self.time_step =0.1
        self.time_step_N=5

        # Initializing Subscribers
        self.state_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.state_sub_callback)
        self.latest_state_msg=None
        self.lidar_sub = rospy.Subscriber('/scan',LaserScan,self.lidar_sub_callback)
        self.latest_lidar_msg=None

        # Initialising publishers
        self.vel_pub = rospy.Publisher('gap_prediction/predicted_gaps_cv_model',Float32MultiArray,queue_size=10)
        self.acc_pub = rospy.Publisher('gap_prediction/predicted_gaps_ca_model',Float32MultiArray,queue_size=10)
        self.real_pub = rospy.Publisher('gap_prediction/real_gaps',Float32MultiArray,queue_size=10)
        self.lidar_pub = rospy.Publisher('gap_prediction/lidar_gaps',Float32MultiArray,queue_size=10)
        # self.pub = rospy.Publisher('simple_string_topic', String, queue_size=10)  # Create a publisher
        
        # Timer which make the node take 'time-step" secs for each iteration
        self.timer = rospy.Timer(rospy.Duration(self.time_step), self.process_message) 

        
    
    def euclidean_distance(self,point1, point2):
        # Calculate Euclidean distance between two points.
        return np.sqrt(np.sum((point1 - point2) ** 2))
    
    def state_sub_callback(self,msg):
        self.latest_state_msg = msg
    
    def lidar_sub_callback(self,msg):
        self.latest_lidar_msg = msg
        # print(self.latest_lidar_msg)

    def real_gap_finder(self,msg):
    #getting actual position of the obstacles
        time = (rospy.Time.now().secs + rospy.Time.now().nsecs/1e9)

        
        pos = [(),(),()]
        for i, name in enumerate(msg.name):
            if name == "cylinder_1":  # Replace with your model's name
                # Extract position (x, y, z) of the model
                pos[0] = np.array([msg.pose[i].position.x,msg.pose[i].position.y])
            if name == "cylinder_2":
                pos[1] = np.array([msg.pose[i].position.x,msg.pose[i].position.y])
            if name == "cylinder_3":
                pos[2] = np.array([msg.pose[i].position.x,msg.pose[i].position.y])   


        self.real_gaps = list(np.array([self.euclidean_distance(pos[0],pos[1]),
                                  self.euclidean_distance(pos[1],pos[2]),
                                  self.euclidean_distance(pos[2],pos[0])]) - 2*self.cylinder_radius)
        self.real_gaps.append(time)

        print(f"Real coordinates: {pos[0],pos[1],pos[2]}")
        print("Real Gaps:",self.real_gaps)
    
    def lidar_callback(self,msg):
        if not(msg): #Sometimes communication with ROS is not established node starts so msg become None type
            return
        
        time = (rospy.Time.now().secs + rospy.Time.now().nsecs/1e9)
        N=self.time_step_N

        '''This pos_object was used to estimate the position using K-means clustering.'''
        # pos_object = Lidar_Pos(msg.ranges,self.cylinder_radius)

        gap_object = Lidar_gaps(msg.ranges,self.cylinder_radius)
        self.lidar_gaps = gap_object.gaps + [time]
        print("Gaps from Lidar:",gap_object.gaps) #Gap from edge points
       

        pos_estimates = gap_object.pos_estimate() #Position estimates from edge_points
        pos_estimates.append(time)
        print("Estimated Coordinates:",pos_estimates)

        self.pos_list.append(pos_estimates)
        self.edge_point_list.append(gap_object.edge_grps)

        if len(self.pos_list)>N+3 and len(self.edge_point_list)>N+3:
            self.pos_list.pop(0)
            self.edge_point_list.pop(0)
            self.cv_model_gaps = gap_object.get_gaps(velocity_model(self.pos_list,self.edge_point_list,time,N))
            self.ca_model_gaps = gap_object.get_gaps(acceleration_model(self.pos_list,self.edge_point_list,time,N))
            self.cv_model_gaps.append(time)
            self.ca_model_gaps.append(time)
            
            print("Velocity Model:",self.cv_model_gaps)
            print("Acceleraiton Model:",self.ca_model_gaps)

        
    def process_message(self,event):
        
        self.real_gap_finder(self.latest_state_msg)
        try:
            self.lidar_callback(self.latest_lidar_msg)
            self.publish_data()
        except:
            print("Object Overlap Occured!!")
            
        print('''
              X==========ONE=====TIME=====STEP=============XX
              ''')
    
    def publish_data(self):
        cv_model_gaps= Float32MultiArray()
        ca_model_gaps= Float32MultiArray()
        lidar_gaps = Float32MultiArray()
        real_gaps = Float32MultiArray()


        cv_model_gaps.data = self.cv_model_gaps
        ca_model_gaps.data = self.ca_model_gaps
        lidar_gaps.data = self.lidar_gaps
        real_gaps.data =self.real_gaps
       

        self.vel_pub.publish(cv_model_gaps)
        self.acc_pub.publish(ca_model_gaps)
        self.lidar_pub.publish(lidar_gaps)
        self.real_pub.publish(real_gaps)


        
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