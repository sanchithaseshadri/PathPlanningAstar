# PathPlanningAstar
An implementation of a mobile robot path planner in Python and ROS

This implementation is essentially a ROS node responsible for robot path planning. It accomplishes the following:
1. Given a map that is an image of the area represented as a ground truth occupancy map (black pixels in places that 
contain obstacles and white pixels where there is free space), this node builds a map that represents the space as a 
dictionary of pixels mapped to boolean values - True if the location is free and False otherwise.
2. Given a source and a destination, it plans a path in steps (step size can be controlled). 
3. Once a path is planned, it can be navigated to using the "Safegoto" module, which is an implementation of the Bug 2 
algorithm. This has obstacle avoidance logic in it.  

