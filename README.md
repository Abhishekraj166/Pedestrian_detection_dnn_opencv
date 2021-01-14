# Pedestrian_detection_dnn_opencv
This code shows the use of Mobile Net Single Shot detector provided with opencv for rosbag data obtained from realsense camera. 

The package loads existing Caffe model from (https://docs.opencv.org/3.3.0/d5/de7/tutorial_dnn_googlenet.html)) googlenet dnn page and use it to classify pedestrian in the camera image topic. 

#Requirements
ROS::Kinect, OpenCV

Steps to follow: 
Launch roscore
* $ roscore

Launch Rosbag node
* $ rosbag play [Bagfile]

Package
* $ cd [ROS Kinetic catkin src folder]
* $ git clone https://github.com/Abhishekraj166/Human_detection.git
* $ cd ..
* $ catkin_make
* $ roslaunch detection_pkg detection_pkg_node

Detected pedestrian is published to /image_converter/output_video
