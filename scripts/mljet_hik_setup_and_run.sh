#!/bin/sh
# Mljet IP=10.0.15.60

export ASV_NAME=mljet
export PLADYPOS_HIKVISION_IP=10.0.15.62
export PLADYPOS_HIKVISION_STREAM=rtsp://stdops:labos123@$PLADYPOS_HIKVISION_IP:554

echo 'Topside hostname: '$HOSTNAME 
echo 'Topside ROS distro:'$ROS_DISTRO
export ROS_MASTER_URI=http://$ASV_NAME:11311/
echo 'ROS master exported to: '$ROS_MASTER_URI
#export ROS_IP=$ROS_IP
#echo 'Exported topside ROS_IP: '$ROS_IP


echo 'Exported ASV name: '$ASV_NAME
echo 'Pinging '$ASV_NAME' ASV'
ping $ASV_NAME -c 3 ;
echo 'Pinging topcam at '$PLADYPOS_HIKVISION_IP 
ping $PLADYPOS_HIKVISION_IP -c 3 ;   
 
echo 'Launching topside GUI...'
roslaunch hikvision_ros start_ip_cam_stream_bttm_side_du.launch

