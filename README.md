# ros2_ws

In `src/realsense_ros2_publisher/src/realsense_publisher.cpp`, change the IP address accordingly to where you are running the `rs-server`.

## Prerequisites

*   `rs-server` should be running on the remote desktop which is connected to the same network as your host PC.
*   `rs-server`'s SDK i.e. RealSense SDK compatible version is 2.53. Please download and install that version of the SDK.

## Brief Description

This ROS2 node retrieves frames from `rs-server` within the same network and publishes intrinsics such as depth, aligned frames, and odometry data into ROS2 topics. These topics are compatible with detection pipelines, RTABMAP (SLAM's pipeline), and obstacle avoidance pipelines.

Remember to run this node before starting any of the pipelines.

## Steps to Run the Package

1.  Run `source source_this.sh` after installing the ROS2 package.
2.  Then, run `./run_realsense_receiver.sh`.

The shell script should be executed after the remote computer's `rs-server` is running.
