# hikvision_ros
Preview bridge node of  hikvision IP camera for ROS with PTZ control interface
 
### Dependences installation

* cv_bridge  ROS Noetic pkg
* image_transport ROS Noetic pkg
* OpenCV4 ROS Noetic pkg
* gstreamer-related libs
* video_stream_opencv ROS Noetic pkg

# ROS Noetic running Python 3 assumed
# Otherwise change ros-distro for ROS1 in the following commands
sudo apt update &&
sudo apt upgrade &&
sudo apt install ros-noetic-vision-opencv &&
sudo apt install ros-noetic-image-transport &&
sudo apt install libopencv-dev python3-opencv &&
python3 -c "import cv2; print(cv2.__version__); exit()" &&
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc \
gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa &&
sudo apt install ros-noetic-video-stream-opencv


### Sourcing ROS and its workspace 
## Topside
echo $ROS_DISTRO
source /opt/ros/noetic/setup.bash
source <path_to_ros_workspace>/devel/setup.bash
# If you're running only one ROS version, it is convinient to 
# add it directly in the ~/.bashrc
sudo echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
sudo echo "source <path_to_ros_workspace>/devel/setup.bash" >> ~/.bashrc

## Bottom-side
ssh <bttm_side_user>@<bttm_side_ip_address> 
# and repeat the same sourcing similarly as for the topside


### Remote access over network in ROS1 
## Adding hostnames on the top- and bottom-side 
## and exporting ROS_MASTER_URI and ROS_IP 

# Top-side 
echo $HOSTNAME 
sudo echo "<bttm_side_ip_address> <bttm_side_hostname>" >> /etc/hosts
export ROS_MASTER_URI=http://<bttm_side_hostname>:11311/
export ROS_IP=<top_side_ip_address> 

# Bottom-side
ssh <bttm_side_user>@<bttm_side_ip_address>
echo $HOSTNAME 
sudo echo "<top_side_ip_address> <top_side_hostname>" >> /etc/hosts
ping <top_side_hostname>
ping <bttm_side_hostname>
export ROS_MASTER_URI=http://<bttm_side_hostname>:11311/
export ROS_IP=<bttm_side_ip_address> 



### Usage 
 
# Direct connection from the operator to the camera
# on the topside w/o bottom-side - runs both the RTSP stream node and the GUI
roslaunch hikvision_ros start start_ip_cam_stream_and_gui.launch


## Separating ROS node launching at the bottom- and top-side
# Connecting the IP cam into ROS @ bottom-side (used for online processing and logging)
ssh <bttm_side_user>@<bttm_side_ip_address>
roslaunch hikvision_ros start start_ip_cam_stream_bttm_side.launch

# Topside (used for surveilance and PTZ control by the operator)
roslaunch hikvision_ros start start_ip_cam_stream_top_side.launch





## usage - OLD



***instant run***

```sh
git clone https://github.com/tanzby/hikvision_ros.git
cd hikvision_ros
mkdir build && cd build
cmake ..
make
```

Publish ` sensor_msgs::Image`

```sh
roscore
source <path/to/devel/setup.sh>
roslaunch hikvision_ros hik.launch
```



***parameters***

You can specify some camera and steam parameters by `hik.launch`

```xml
<arg name="ip_addr" default="192.168.5.100"/>
<arg name="user_name" default="admin"/>
<arg name="password" default="admin"/>
<arg name="port" default="8000"/>
<arg name="channel" default="1"/>
```

Or in command line

```sh
roslaunch hikvision_ros hik.launch ip_addr:=192.168.5.100 password:=123456
```



***support for camera_calibration***

you can use [camera_calibration](http://wiki.ros.org/camera_calibration/)  to calibrate hikvision camera, **hik_ros**  provides *set_camera_info* sevice for receiving and storing camera's calibration parameters. 

***example***

```sh
# open camera
roslaunch hikvision_ros hik.launch

# see topic name
rostopic list

# [output]
# ➜  ~ rostopic list
# /hik_cam_node/camera_info
# /hik_cam_node/hik_camera
# ...

# calibrate
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/hik_cam_node/hik_camera  camera:=/hik_cam_node/hik_camera
```

then begin calibration. After calibration you straightly press **commit** button,  **hik_ros** has the ability to save the calibration parameter to `camera_info_url`, which is set in launch file OR use default path (  `~/.ros/camera_info` )   

```sh
# [output]
#[ INFO] [1551582484.454024618]: New camera info received
#[ INFO] [1551582484.454296067]: [ hik_camera ] Write camera_info to ~/.ros/camera_info/hik_camera.yaml success.
```

