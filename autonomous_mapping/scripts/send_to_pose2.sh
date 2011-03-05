#!/bin/bash
rostopic pub /robot_pose geometry_msgs/Pose """
position:
   x: -3.614
   y: -0.113
   z: 0.05 
orientation:
   x: -0.006
   y: 0.002
   z: 0.726
   w: 0.688
"""