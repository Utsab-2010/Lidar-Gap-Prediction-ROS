import math 
import numpy as np

def velocity_model(pos_list,edge_point_list,time,N):

    time = time-pos_list[-1-N][-1] # calculating the prediction time_period

    del_t = pos_list[-1-N][-1]- pos_list[-2-N][-1]

    # Calculate the avg velocity of particle N time_steps in the past 
    vel_1 = ((pos_list[-1-N][0][0]-pos_list[-2-N][0][0])/del_t , (pos_list[-1-N][0][1]-pos_list[-2-N][0][1])/del_t )
    vel_2 = ((pos_list[-1-N][1][0]-pos_list[-2-N][1][0])/del_t , (pos_list[-1-N][1][1]-pos_list[-2-N][1][1])/del_t )
    vel_3 = ((pos_list[-1-N][2][0]-pos_list[-2-N][2][0])/del_t , (pos_list[-1-N][2][1]-pos_list[-2-N][2][1])/del_t )

    # Calculating the net change in position
    del_p1 = time*np.array(vel_1)
    del_p2 = time*np.array(vel_2)
    del_p3 = time*np.array(vel_3)

    predicted_points = [None,None,None]
    # Calculating the new edge points
    predicted_points[0] = edge_point_list[-1-N][0]+ del_p1
    predicted_points[1] = edge_point_list[-1-N][1]+ del_p2
    predicted_points[2] = edge_point_list[-1-N][2]+ del_p3

    # print(predicted_points)
    return predicted_points

def acceleration_model(pos_list,edge_point_list,time,N):
    time = time-pos_list[-1-N][-1]
    del_t1 = pos_list[-1-N][-1]- pos_list[-2-N][-1]

    # Calculate the avg velocity of objects N time_steps in the past 
    vel_11 = ((pos_list[-1-N][0][0]-pos_list[-2-N][0][0])/del_t1 , (pos_list[-1-N][0][1]-pos_list[-2-N][0][1])/del_t1 )
    vel_12 = ((pos_list[-1-N][1][0]-pos_list[-2-N][1][0])/del_t1 , (pos_list[-1-N][1][1]-pos_list[-2-N][1][1])/del_t1 )
    vel_13 = ((pos_list[-1-N][2][0]-pos_list[-2-N][2][0])/del_t1 , (pos_list[-1-N][2][1]-pos_list[-2-N][2][1])/del_t1 )

    del_t2 = pos_list[-2-N][-1]- pos_list[-3-N][-1]
    
    # Calculate the avg velocity of objects N+1 time_steps in the past 
    vel_21 = ((pos_list[-2-N][0][0]-pos_list[-3-N][0][0])/del_t2 , (pos_list[-2-N][0][1]-pos_list[-3-N][0][1])/del_t2 )
    vel_22 = ((pos_list[-2-N][1][0]-pos_list[-3-N][1][0])/del_t2 , (pos_list[-2-N][1][1]-pos_list[-3-N][1][1])/del_t2 )
    vel_23 = ((pos_list[-2-N][2][0]-pos_list[-3-N][2][0])/del_t2 , (pos_list[-2-N][2][1]-pos_list[-3-N][2][1])/del_t2 )

    # Calculate the avg acceleration of objects N time_steps in the past 
    acc_1 = (np.array(vel_11)-np.array(vel_21))/((del_t1+del_t2)/2)
    acc_2 = (np.array(vel_12)-np.array(vel_22))/((del_t1+del_t2)/2)
    acc_3 = (np.array(vel_13)-np.array(vel_23))/((del_t1+del_t2)/2)
    
    # Calculating the net change in position
    del_p1 = time*np.array(vel_11) + 0.5*np.array(acc_1)*(time**2)
    del_p2 = time*np.array(vel_12) + 0.5*np.array(acc_2)*(time**2)
    del_p3 = time*np.array(vel_13) + 0.5*np.array(acc_3)*(time**2)

    predicted_points = [None,None,None]
    # Calculating the new edge points
    predicted_points[0] = edge_point_list[-1-N][0]+ del_p1
    predicted_points[1] = edge_point_list[-1-N][1]+ del_p2
    predicted_points[2] = edge_point_list[-1-N][2]+ del_p3

    return predicted_points