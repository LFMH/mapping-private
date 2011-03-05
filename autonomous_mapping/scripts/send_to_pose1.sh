#!/bin/bash
rostopic pub /robot_pose geometry_msgs/Pose """
position:
   x: -0.636
   y: 1.011
   z: 0.05 
orientation:
   x: -0.014
   y: 0.013
   z: -0.060
   w: 1.0
"""