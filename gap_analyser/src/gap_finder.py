import numpy as np

class Lidar_gaps:
    def __init__(self,lidar_data,radius):
        
        self.radius = radius
        self.lidar_data = np.array(lidar_data)

        # Finding the edge lidar points of the objects 
        finite_indices = np.where(np.isfinite(self.lidar_data))[0]
        inf_indices = np.where(np.isinf(self.lidar_data))[0]
        self.edge_indices = np.array([
        idx for idx in finite_indices
        if any(abs(idx - inf_idx) == 1 for inf_idx in inf_indices)
        ])
        
        # Getting an array of the corner points(6 points for 3 objects)
        self.edge_data = self.lidar_data[self.edge_indices] 
        self.x = (self.radius+self.edge_data)*np.cos(self.edge_indices*3.14/180)
        self.y = (self.radius+self.edge_data)*np.sin(self.edge_indices*3.14/180)
        self.edge_points = np.column_stack((self.x,self.y))

        # Grouping the corner points in to 3 groups in order 
        self.edge_grps = self.arrange_data(self.edge_points)
        # print(self.edge_grps)
        self.gaps=self.get_gaps()
        

    def euclidean_distance(self,point1, point2):
        """Calculate Euclidean distance between two points."""
        return np.sqrt(np.sum((point1 - point2) ** 2))
    

    def get_gaps(self,edge_grps=None):
        '''Function to get gaps between lidar edge points(both predicted and recorded ones)'''
        # print("hi")
        if not(edge_grps):
            edge_grps = self.edge_grps

        gaps=[None,None,None]
        gaps[0] = self.euclidean_distance(edge_grps[0][1],edge_grps[1][0])
        gaps[1] = self.euclidean_distance(edge_grps[1][1],edge_grps[2][0])
        gaps[2] = self.euclidean_distance(edge_grps[2][1],edge_grps[0][0])
        # print()
        return gaps

    def arrange_data(self,data):
        if self.euclidean_distance(data[0],data[-1]) <= 2.4*self.radius:
        # Move the first element to the back
            # print("rearranged")
            data = np.vstack([data[1:], data[0]])
        
        #calculate the distance of their mid_points from the lidar
        means = np.mean(data.reshape(3, 2, 2), axis=1) # This throws an error during object overlap which is captured by the try and except block in the main file
        distances = np.linalg.norm(means, axis=1)
    
        # Get the indices that would sort the distances from farthest to closest
        ranks = np.argsort(-distances)
        
        
        grp_arr = [None,None,None]    
        # Group the edge points into 3 grps for the 3 obstacles
        for rank,i in enumerate(ranks):
            grp_arr[rank] = np.array(data[2*i:2*i+2])
            
        return grp_arr
    
    def pos_estimate(self):
        position = [None,None,None]
        for i in range(3):
            position[i] =(self.edge_grps[i][0]+self.edge_grps[i][1])/2

        return position