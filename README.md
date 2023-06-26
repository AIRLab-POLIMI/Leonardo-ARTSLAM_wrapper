# ARTSLAM_WRAPPER
***artslam_wrapper*** is a ROS wrapper for [ART-SLAM](https://github.com/MatteoF94/ARTSLAM). Right now, it accepts only point clouds as input, and it performs LiDAR SLAM, as intended. It also outputs, periodically, the built 3D map and the estimated positions of the robot (respectively on the /artslam_laser_3d_wrapper/single_cloud and /artslam_laser_3d_wrapper/markers topics).

## Instructions
The file src/artslam_controller.cpp describes in detail how to build your own SLAM system, step by step. If you want to perform SLAM offline (no bags), do the following service call:
```bash
rosservice call /artslam_controller/OfflineSLAM
```
