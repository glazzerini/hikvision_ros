#ifndef HIK_CAM_H
#define HIK_CAM_H

#include <iostream>
#include <string>

#include "hikvision_sdk/HCNetSDK.h"
#include "hikvision_sdk/PlayM4.h"
#include "hikvision_sdk/error.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

class HikvisionCamera
{
private:

    /// LABUST-related ROS code
    ros::Subscriber ptz_controls_sub;
    ros::Subscriber ptz_key_teleop_controls_sub;
    ros::Subscriber ptz_joy_teleop_controls_sub;

    std::string PTZ_CONTROLS_TOPIC;
    std::string PTZ_KEYBOARD_TELEOP_CONTROLS_TOPIC;
    std::string PTZ_JOYSTICK_TELEOP_CONTROLS_TOPIC;
    int ROS_RATE;
    ros::NodeHandle priv_node;
    ros::NodeHandle pub_node;
    int ptz_state;
    static const int PTZ_CONTROLS_STOPPED = 0;
    static const int PAN_RIGHT_CMD=1;
    static const int PAN_LEFT_CMD=2;
    static const int TILT_UP_CMD=3;
    static const int TILT_DOWN_CMD=4;
    static const int ZOOM_CONTROLS_STOPPED = 5;
    static const int ZOOM_IN_CMD=6;
    static const int ZOOM_OUT_CMD=7;
    static const int FOCUS_CONTROLS_STOPPED = 8;
    static const int FOCUS_FAR_CMD = 9;
    static const int FOCUS_NEAR_CMD = 10;
    static const int IRIS_CONTROLS_STOPPED = 11;
    static const int IRIS_OPEN_CMD = 12;
    static const int IRIS_CLOSE_CMD = 13;


    /// ros parameters
    image_transport::CameraPublisher image_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr;
    ros::ServiceServer SetCameraInfoSrv;




    /// camera parameters
    int image_width;
    int image_height;

    std::string ip_addr;
    std::string usr_name;
    std::string password;
    std::string frame_id;
    std::string camera_name;
    std::string camera_info_url;

    int port;
    int channel;
    int link_mode;

    LONG user_id;
    LONG data_play_handler;



    static void decodeCallback_(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo, void *nUser, int nReserved2)
    {
        ((HikvisionCamera *) nUser)->decodeCallback(nPort, pBuf, nSize, pFrameInfo);
    }

    static void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
    {
        ((HikvisionCamera *) pUser)->dataCallback(lRealHandle, dwDataType, pBuffer, dwBufSize);
    }

    std::string expandUserPath(std::string path);

    void decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo);

    void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize);

    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);


    bool initHikSDK();
    void initializeHikvisionCameraAtributes();
    void initializeROSParameters();
    void initializeROSSubscribers();
    void initializeROSPublishers();
    void initializeROSServices();

    void ptzControlsCallback(const std_msgs::Int32::ConstPtr& msg);
    void ptzKeyboardControlsCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void ptzJoystickControlsCallback(const geometry_msgs::Twist::ConstPtr& msg);


public:



    HikvisionCamera();

    ~HikvisionCamera();

    void run();



};

#endif //HIK_CAM_H
