# Lidar-Gap-Prediction-ROS

**Documentation:** [Dynamic Gap Analysis and Prediction in ROS](https://spiky-cricket-14b.notion.site/Dynamic-Gap-Analysis-and-Prediction-in-ROS-172033ce2ca58069a99cf2e3d667d210)

### Requirements:
1. ROS Noetic
2. Ubuntu 20.04 or other Noetic Compatible distros
3. Turtlebot3 packages and dependencies
4. Gazebo Simulator(is installed with Noetic)

### Important Node and Launch Files
1. gap_environment.launch
2. gap_predictor.py
3. rviz_markers.py

### Module Files
1. obstacle_movement.py
2. prediction_models.py
3. gap_finder.py
4. kmeans_clustering.py (works but not used)
5. dbscan_clustering.py (works but not used)

Both the clustering module files are functional but generated sudden errors due to loop-holes in their algorithm. Remaining scripts are useless test scripts.