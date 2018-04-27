# autonomous_vehicle
SNU ZERO repo for autonomous vehicle competition

# ROS




# occupancy_map & monitor_map color code
R == 255: this is occupied
R != 255: this is free region
B == 255: this is the target point
G == 255: estimated path points (monitor_map only)

(R,G,B) ==(200,200,200): this is lidar position

# core_msgs
Path3DArray: for path data,
delta: the delta that the hybrid A* used for the current fragment of the path,
this can be used for estimation of curvature of the path

# smoother
let's use curvature Cost for estimation of path curvature

Hi
