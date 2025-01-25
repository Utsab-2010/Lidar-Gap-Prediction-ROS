import numpy as np

class DBS_estimation:
    def __init__(self,lidar_data,radius):
        self.radius = radius
        self.lidar_data = np.array(lidar_data)
        self.finite_data = self.lidar_data[np.isfinite(self.lidar_data)]
        # print(self.finite_data)
        
        self.finite_indices = np.where(np.isfinite(self.lidar_data))[0]
        # print(self.finite_indices)
        self.x = (self.radius+self.finite_data)*np.cos(self.finite_indices*3.14/180)
        self.y = (self.radius+self.finite_data)*np.sin(self.finite_indices*3.14/180)
        self.points = np.column_stack((self.x,self.y))
        


        # print(self.points)
        
    def euclidean_distance(self,point1, point2):
        """Calculate Euclidean distance between two points."""
        return np.sqrt(np.sum((point1 - point2) ** 2))

    def region_query(self,data, point_idx, epsilon):
        """Find all neighbors of a point within epsilon distance."""
        neighbors = []
        for idx, point in enumerate(data):
            if self.euclidean_distance(data[point_idx], point) <= epsilon:
                neighbors.append(idx)
        return neighbors


    def dbscan(self,data, epsilon, min_pts):
        """Main DBSCAN algorithm."""
        labels = np.full(len(data),None,dtype=object)  # Initialize all points as unvisited (label 0)
        cluster_id = 0
        
        for point_idx in range(len(data)):
            if labels[point_idx] != None:  # Skip already visited points
                continue

            neighbours = self.region_query(data,point_idx,epsilon)

            if len(neighbours) < min_pts:
                labels[point_idx]=-1
                continue

            cluster_id+=1

            # print(neighbours)
            seed_set = neighbours
            seed_set.remove(point_idx)
            # print(seed_set)

            for s_indx in seed_set:
                if labels[s_indx]==-1:
                    labels[s_indx]=cluster_id
                if labels[s_indx]!=None:
                    continue
                labels[s_indx]=cluster_id
                s_neighbours = self.region_query(data,s_indx,epsilon)
                if len(s_neighbours)>=min_pts:
                    seed_set.extend(s_neighbours)
                    seed_set = list(dict.fromkeys(seed_set))
        
        return labels
    
    def pos_estimate(self,epsilon,min_pts):
        labels = self.dbscan(self.points,epsilon,min_pts)
        print("Labels:",labels)
        grp_1 = self.points[np.where(labels==1)]
        grp_2 = self.points[np.where(labels==2)]
        grp_3 = self.points[np.where(labels==3)]

        p1 = (np.mean(grp_1[:,0]),np.mean(grp_1[:1]))
        p2 = (np.mean(grp_2[:,0]),np.mean(grp_2[:1]))
        p3 = (np.mean(grp_3[:,0]),np.mean(grp_3[:1]))

        return (p1,p2,p3)

# Example 2D dataset
# data = np.array([
#     [1, 2], [1.1, 2.1], [5, 5], [5.1, 5.2], [10, 10],
#     [10.1, 10.2], [10.2, 10.3], [0.5, 0.6]
# ])

# # DBSCAN parameters
# epsilon = 1.0
# min_pts = 2

# # Run DBSCAN
# labels = dbscan(data, epsilon, min_pts)
# print("Cluster Labels:", labels)
