import numpy as np

class Lidar_Pos:
    def __init__(self,lidar_data,radius):
        self.radius = radius
        self.lidar_data = np.array(lidar_data)
        self.finite_data = self.lidar_data[np.isfinite(self.lidar_data)]
        self.finite_indices = np.where(np.isfinite(self.lidar_data))[0]
        self.x = (self.radius+self.finite_data)*np.cos(self.finite_indices*3.14/360)
        self.y = (self.radius+self.finite_data)*np.sin(self.finite_indices*3.14/360)
        self.points = np.column_stack((self.x,self.y))
        print(self.points)
        
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

    def expand_cluster(self,data, labels, point_idx, cluster_id, epsilon, min_pts):
        """Expand the cluster recursively."""
        neighbors = self.region_query(data, point_idx, epsilon)
        if len(neighbors) < min_pts:
            labels[point_idx] = -1  # Mark as noise
            return False
        else:
            labels[point_idx] = cluster_id
            i = 0
            while i < len(neighbors):
                neighbor_idx = neighbors[i]
                if labels[neighbor_idx] == 0:  # Unvisited point
                    labels[neighbor_idx] = cluster_id
                    new_neighbors = self.region_query(data, neighbor_idx, epsilon)
                    if len(new_neighbors) >= min_pts:
                        neighbors += new_neighbors
                elif labels[neighbor_idx] == -1:  # Previously labeled as noise
                    labels[neighbor_idx] = cluster_id
                i += 1
            return True


    def dbscan(self,data, epsilon, min_pts):
        """Main DBSCAN algorithm."""
        labels = np.zeros(len(data))  # Initialize all points as unvisited (label 0)
        cluster_id = 0
        
        for point_idx in range(len(data)):
            if labels[point_idx] != 0:  # Skip already visited points
                continue
            if self.expand_cluster(data, labels, point_idx, cluster_id + 1, epsilon, min_pts):
                cluster_id += 1  # Start a new cluster
        
        return labels
    
    def pos_estimate(self,epsilon,min_pts):
        labels = self.dbscan(self.points,epsilon,min_pts)
        print("Labels:",labels)
        grp_1 = self.points[np.where(labels==1)]
        grp_2 = self.points[np.where(labels==2)]
        grp_3 = self.points[np.where(labels==3)]

        p1 = (np.mean(grp_1[:,0]),np.mean(grp_1[:,1]))
        p2 = (np.mean(grp_2[:,0]),np.mean(grp_2[:,1]))
        p3 = (np.mean(grp_3[:,0]),np.mean(grp_3[:,1]))

        return (p1,p2,p3)

# Example 2D dataset
data = np.array([
    [1, 2], [1.1, 2.1], [5, 5], [5.1, 5.2], [10, 10],
    [10.1, 10.2], [10.2, 10.3], [0.5, 0.6]
])

# # DBSCAN parameters
# epsilon = 1.0
# min_pts = 2

# # Run DBSCAN
# labels = dbscan(data, epsilon, min_pts)
# print("Cluster Labels:", labels)
