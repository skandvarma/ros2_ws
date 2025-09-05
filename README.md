in this src/realsense_ros2_publisher/src/realsense_publisher.cpp change the IP 
address accordingly to which you run the rs-server on. 

rs-server should be run on the remote desktop which is connected to the same network as your host PC

rs-server's SDK i.e realsense SDK compatible version is 2.53 please download and install that version of SDK

What this ros2 node do is get the frames from rs-server from the same network and publish those intrinsics such as depth , aligned frames , and odometry data into ros2 topics which is accepted by the detection pipeline , RTABMAP i.e SLAM's pipeline as well as obstacle avoidance pipeline 

Remember to run this node before you the any of the pipelines 
