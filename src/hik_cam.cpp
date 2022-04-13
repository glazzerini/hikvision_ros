#include "hik_cam.h"


std::string HikvisionCamera::expandUserPath(std::string path)
{
    if (not path.empty() and path[0] == '~') {
        assert(path.size() == 1 or path[1] == '/');  // or other error handling
        char const* home = getenv("HOME");
        if (home or ((home = getenv("USERPROFILE")))) {
            path.replace(0, 1, home);
        }
        else {
            char const *hdrive = getenv("HOMEDRIVE"),
                    *hpath = getenv("HOMEPATH");
            assert(hdrive);  // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}

void HikvisionCamera::decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo)
{
    int lFrameType = pFrameInfo->nType;

    if (lFrameType == T_YV12)
    {

        cv::Mat picBGR;
        cv::Mat picYV12 = cv::Mat(pFrameInfo->nHeight * 3/2, pFrameInfo->nWidth, CV_8UC1, pBuf);
        cv::cvtColor(picYV12, picBGR, cv::COLOR_YUV2BGR_YV12);

        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info_mgr->getCameraInfo()));
        ci->header = camera_info_mgr->getCameraInfo().header;
        ci->header.stamp = ros::Time::now();

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(ci->header, "bgr8", picBGR).toImageMsg();

        // publish the image
        image_pub.publish(*msg, *ci);

#ifndef NDEBUG
        // ROS_INFO("[%s] Stream CallBack, Convert YV12 to sensor_msgs.",camera_name.c_str());
#endif
    }
}

void HikvisionCamera::dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize)
{
    int dRet = 0;
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD:    //system head
    {
        if (!PlayM4_GetPort(&port)) //unavailable lPort
        {
            break;
        }
        if (dwBufSize > 0)
        {
            if (!PlayM4_SetStreamOpenMode(port, STREAME_REALTIME))  //set strean mode
            {
                break;
            }
            if (!PlayM4_OpenStream(port, pBuffer, dwBufSize, image_width * image_height))
            {
                dRet = PlayM4_GetLastError(port);
                break;
            }
            //decoding callback
            if (!PlayM4_SetDecCallBackMend(port, decodeCallback_, this))
            {
                dRet = PlayM4_GetLastError(port);
                break;
            }
            //open decoding
            if (!PlayM4_Play(port, 0)) // play the video stream
            {
                dRet = PlayM4_GetLastError(port);
                break;
            }
        }
    } break;


    case NET_DVR_STREAMDATA:
    {
        if (dwBufSize > 0 && port != -1)
        {
            if (!PlayM4_InputData(port, pBuffer, dwBufSize))
            {
                break;
            }
        }
    } break;

    default:
    {
        if (dwBufSize > 0 && port != -1)
        {
            if (!PlayM4_InputData(port, pBuffer, dwBufSize))
            {
                break;
            }
        }
    } break;
    }
    if(dRet != 0){
        ROS_INFO("[%s] Error Occurs when data callback, get: %s",camera_name.c_str(),
                 get_error_str(NET_DVR_GetLastError()).c_str());
    }
}

bool HikvisionCamera::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                                    sensor_msgs::SetCameraInfo::Response& rsp)
{
    ROS_INFO("New camera info received");
    sensor_msgs::CameraInfo &info = req.camera_info;
    info.header.frame_id = frame_id;

    if (info.width != image_width || info.height != image_height) {
        rsp.success = 0;
        char buf[100];
        sprintf(buf,"Camera_info resolution %ix%i does not match current video "
                    "setting, camera running at resolution %ix%i.",info.width,info.height,image_width,image_height);
        rsp.status_message = buf;
        ROS_ERROR("%s", rsp.status_message.c_str());
        return true;
    }
    if(camera_info_url.empty())
        camera_info_url = expandUserPath("~/.ros/camera_info/" + camera_name + ".yaml");

    if(!camera_calibration_parsers::writeCalibration(camera_info_url,camera_name,info))
    {
        rsp.status_message = "Error formatting camera_info for storage. filename = "+camera_info_url;
        rsp.success = 0;
    }
    else
    {
        rsp.success = 1;
        rsp.status_message = "Write camera_info to "+camera_info_url+" success.";
    }

    if (!rsp.success) {
        ROS_ERROR("[ %s ] %s",camera_name.c_str(), rsp.status_message.c_str());
    }
    else{
        ROS_INFO("[ %s ] %s",camera_name.c_str(), rsp.status_message.c_str());
    }
    return true;
}

bool HikvisionCamera::initHikSDK()
{
    NET_DVR_Init();
#ifndef NDEBUG
    NET_DVR_SetLogToFile(3, "./hiklog");
#endif
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40  struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort = port;
    memcpy(struLoginInfo.sDeviceAddress,ip_addr.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, usr_name.c_str(), NAME_LEN);
    memcpy(struLoginInfo.sPassword, password.c_str(), NAME_LEN);
    user_id = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

    if (user_id < 0)
    {
        ROS_ERROR("[%s] Login fail, get: %s",camera_name.c_str(), get_error_str(NET_DVR_GetLastError()).c_str());
        return false;
    }

    //Set callback function of getting stream.
    NET_DVR_PREVIEWINFO struPlayInfo = {0};
    struPlayInfo.hPlayWnd     = 0;
    struPlayInfo.lChannel     = channel;  //channel NO
    struPlayInfo.dwLinkMode   = link_mode;
    struPlayInfo.bBlocked     = 1;
    struPlayInfo.dwDisplayBufNum = 1;

    data_play_handler = NET_DVR_RealPlay_V40(user_id, &struPlayInfo, nullptr, nullptr);

    if (data_play_handler < 0)
    {
        int i = NET_DVR_GetLastError();
        if(i == 0)
        {
            sleep(1);
            data_play_handler = NET_DVR_RealPlay_V40(user_id, &struPlayInfo, nullptr, nullptr);
        }
        if (data_play_handler < 0) {
            ROS_ERROR("[%s] Get RealPlay Handler error, get: %s",camera_name.c_str(),
                      get_error_str(NET_DVR_GetLastError()).c_str());
            return false;
        }
    }
    ROS_INFO("[%s] Set Play Handler successully.",camera_name.c_str());


    //Set callback function of getting stream.
    // Commented out since video_stream_opencv ROS node will stream video
    /*int iRet = NET_DVR_SetStandardDataCallBackEx(data_play_handler, dataCallback, this);
    if (!iRet)
    {
        ROS_ERROR("[%s] Set DataCall error, get: %s", camera_name.c_str(), get_error_str(NET_DVR_GetLastError()).c_str());
        return false;
    }
    else ROS_INFO("[%s] Set Data Callback successully.",camera_name.c_str());*/

       return true;
}


void HikvisionCamera::ptzControlsCallback(const std_msgs::Int32::ConstPtr& msg){
    int control_code = msg->data;
    // ROS_INFO("I heard ptz Controls code: [%d]", msg->data());

    switch (control_code) {
    case 0:
        NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 0);
        NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 0);
        NET_DVR_PTZControl(data_play_handler, TILT_UP, 0);
        NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 0);
        //ROS_INFO("Stopping all PTZ controls...");
        break;
    case 1:
        NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 1);
        //ROS_INFO("Panning right...");
        sleep(0.1);
        NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 0);
        break;
    case 2:
        NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 1);
        //ROS_INFO("Panning left...");
        sleep(0.1);
        NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 0);
        break;
    case 3:
        NET_DVR_PTZControl(data_play_handler, TILT_UP, 1);
        //ROS_INFO("Tilting up...");
        sleep(0.005);
        NET_DVR_PTZControl(data_play_handler, TILT_UP, 0);
        break;
    case 4:
        NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 1);
        //ROS_INFO("Tilting down...");
        sleep(0.1);
        NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 0);
        break;
    default:
        break;
    }
 //   sleep(0.1);
}


void HikvisionCamera::ptzKeyboardControlsCallback(const geometry_msgs::Twist::ConstPtr& msg){
    int pan = msg->angular.z;
    int tilt = msg->linear.x;
    int zoom = msg->linear.y;
    int focus = msg->linear.z;
    int iris = msg->angular.x;

   // ROS_INFO("PTZ state: %d", ptz_state);

    //int control_code(0);
    // ROS_INFO("I heard ptz Controls code: [%d]", msg->data());

    if ( pan > 0 & tilt==0) {
//        control_code = PAN_LEFT_CMD;
        NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 0);
        //ROS_INFO("Panning left...");
        ptz_state = PAN_LEFT_CMD;
    }
    else if (pan < 0 & tilt==0) {
//        control_code = PAN_RIGHT_CMD;
        NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 0);
        //ROS_INFO("Panning right...");
        ptz_state = PAN_RIGHT_CMD;
    }
    else if ( pan==0 & tilt==0 & ( ptz_state == PAN_LEFT_CMD  | ptz_state == PAN_RIGHT_CMD  )){
        if ( ptz_state == PAN_LEFT_CMD  ) {
            NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 1);
            //ROS_INFO("Stopping panning left...");
            ptz_state = PTZ_CONTROLS_STOPPED;
        }
        else if ( ptz_state == PAN_RIGHT_CMD  ){
            NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 1);
            //ROS_INFO("Stopping panning right...");
            ptz_state = PTZ_CONTROLS_STOPPED;
        }
    }

    if ( tilt>0 & pan==0 ){
        NET_DVR_PTZControl(data_play_handler, TILT_UP, 0);
        //ROS_INFO("Tilting up...");
        ptz_state = TILT_UP_CMD;
    }
    else if ( tilt<0 & pan==0 ) {
        NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 0);
        //ROS_INFO("Tilting down...");
        ptz_state = TILT_DOWN_CMD;
    }
    else if ( tilt==0 & pan==0 & (ptz_state == TILT_UP_CMD | ptz_state==TILT_DOWN_CMD)) {
        if ( ptz_state == TILT_UP_CMD  ){
            for (int i=1; i<10; i++) NET_DVR_PTZControl(data_play_handler, TILT_UP, 1);
            //ROS_INFO("Stopping tilting up...");
            ptz_state = PTZ_CONTROLS_STOPPED;
        }
        else if (ptz_state==TILT_DOWN_CMD){
            for (int i=1; i<10; i++) NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 1);
           //ROS_INFO("Stopping tilting down...");
            ptz_state = PTZ_CONTROLS_STOPPED;

        }

//        if ( ptz_state == PTZ_CONTROLS_STOPPED) {
//            NET_DVR_PTZControl(data_play_handler, PAN_RIGHT, 0);
//            NET_DVR_PTZControl(data_play_handler, PAN_LEFT, 0);
//            NET_DVR_PTZControl(data_play_handler, TILT_UP, 0);
//            NET_DVR_PTZControl(data_play_handler, TILT_DOWN, 0);
//            ROS_INFO("Stopping all PTZ controls...");
//        }
    }

    if ( zoom > 0 ) {
        NET_DVR_PTZControl(data_play_handler, ZOOM_IN, 0);
        //ROS_INFO("Zooming in...");
        ptz_state = ZOOM_IN_CMD;
    }
    else if (zoom < 0 ) {
        NET_DVR_PTZControl(data_play_handler, ZOOM_OUT, 0);
        //ROS_INFO("Zooming out...");
        ptz_state = ZOOM_OUT_CMD;
    }
    else if ( zoom==0 & ptz_state==ZOOM_IN_CMD ){
        NET_DVR_PTZControl(data_play_handler, ZOOM_IN, 1);
        //ROS_INFO("Stopping zooming in...");
        ptz_state = ZOOM_CONTROLS_STOPPED;
    }
    else if ( zoom==0 &  ptz_state==ZOOM_OUT_CMD) {
        NET_DVR_PTZControl(data_play_handler, ZOOM_OUT, 1);
        //ROS_INFO("Stopping zooming out...");
        ptz_state = ZOOM_CONTROLS_STOPPED;
    }

    if ( focus > 0 ) {
        NET_DVR_PTZControl(data_play_handler, FOCUS_FAR, 0);
        //ROS_INFO("Focusing far...");
        ptz_state = FOCUS_FAR_CMD;
    }
    else if (focus < 0 ) {
        NET_DVR_PTZControl(data_play_handler, FOCUS_NEAR, 0);
        //ROS_INFO("Focusing near...");
        ptz_state = FOCUS_NEAR_CMD;
    }
    else if ( focus==0 & ptz_state==FOCUS_FAR ) {
        NET_DVR_PTZControl(data_play_handler, FOCUS_FAR, 1);
        //ROS_INFO("Stopping focusing far...");
        ptz_state = FOCUS_CONTROLS_STOPPED;
    }
    else if ( focus==0 &  ptz_state==FOCUS_NEAR) {
        NET_DVR_PTZControl(data_play_handler, FOCUS_NEAR, 1);
        //ROS_INFO("Stopping focusing near...");
        ptz_state = FOCUS_CONTROLS_STOPPED;
    }

    if ( iris > 0 ) {
        NET_DVR_PTZControl(data_play_handler, IRIS_OPEN, 0);
        //ROS_INFO("Opening iris...");
        ptz_state = IRIS_OPEN_CMD;
    }
    else if (iris < 0 ) {
        NET_DVR_PTZControl(data_play_handler, IRIS_CLOSE, 0);
        //ROS_INFO("Closing iris...");
        ptz_state = IRIS_CLOSE_CMD;
    }
    else if ( iris==0 & ptz_state==IRIS_OPEN_CMD ) {
        NET_DVR_PTZControl(data_play_handler, IRIS_OPEN, 1);
        //ROS_INFO("Stopping openning iris...");
        ptz_state = FOCUS_CONTROLS_STOPPED;
    }
    else if ( iris==0 &  ptz_state==IRIS_CLOSE_CMD) {
        NET_DVR_PTZControl(data_play_handler, IRIS_CLOSE, 1);
        //ROS_INFO("Stopping closing iris...");
        ptz_state = FOCUS_CONTROLS_STOPPED;
    }



}

void HikvisionCamera::ptzJoystickControlsCallback(const geometry_msgs::Twist::ConstPtr& msg){
}

void HikvisionCamera::initializeHikvisionCameraAtributes(){
    ptz_state = PTZ_CONTROLS_STOPPED; // stop all ptz controls
}

void HikvisionCamera::initializeROSParameters(){
    // LABUST-related code
    priv_node.getParam("PTZ_CONTROLS_TOPIC", PTZ_CONTROLS_TOPIC); //,"ptz_setpoints");
    priv_node.getParam("PTZ_KEYBOARD_TELEOP_CONTROLS_TOPIC", PTZ_KEYBOARD_TELEOP_CONTROLS_TOPIC); //,"ptz_setpoints");
    priv_node.getParam("PTZ_JOYSTICK_TELEOP_CONTROLS_TOPIC", PTZ_JOYSTICK_TELEOP_CONTROLS_TOPIC); //,"ptz_setpoints");

    priv_node.getParam("ROS_RATE",ROS_RATE); //,10);

//    ROS_INFO("Ros TOPIC: %s", PTZ_CONTROLS_TOPIC.c_str());
//    ROS_INFO("Ros rate: %d", ROS_RATE);


    /// camera parameter
    priv_node.getParam("camera_frame_id", frame_id); //, std::string("hik_camera"));
    priv_node.getParam("camera_name", camera_name); //,  std::string("hik_camera"));
    priv_node.getParam("camera_info_url", camera_info_url); //, std::string(""));

    priv_node.getParam("ip_addr", ip_addr); //,"192.168.5.100");
    ROS_INFO("[%s] ip address:\t%s", camera_name.c_str(), ip_addr.c_str());

    priv_node.getParam("user_name",usr_name); //,"admin");
    ROS_INFO("[%s] user name: \t%s", camera_name.c_str(), usr_name.c_str());

    priv_node.getParam("password",password); // ,"ht123456");
    ROS_INFO("[%s] password:  \t%s", camera_name.c_str(), password.c_str());

    priv_node.getParam("port",port); //, 8000);
    ROS_INFO("[%s] port:      \t%d", camera_name.c_str(), port);

    priv_node.getParam("channel",channel); //,1);
    ROS_INFO("[%s] channel:   \t%d", camera_name.c_str(), channel);

    priv_node.getParam("link_mode",link_mode); //, 0);
    if(link_mode < 0 || link_mode >5)
    {
        ROS_WARN("[%s] value %d for link_mode is illegal, set to default value 0 (tcp)",camera_name.c_str(), link_mode);
    }

    std::string _mode []  = {"tcp", "udp", "multicast","rtp","rtp/rtsp", "rstp/http"};
    ROS_INFO("[%s] link mode: \t%s", camera_name.c_str(), _mode[link_mode].c_str());

    priv_node.getParam("image_width",image_width) ; //,1280);
    ROS_INFO("[%s] image width:  \t%d", camera_name.c_str(), image_width);

    priv_node.getParam("image_height",image_height); //, 720);
    ROS_INFO("[%s] image height: \t%d", camera_name.c_str(), image_height);

}

void HikvisionCamera::initializeROSSubscribers()
{
    ROS_INFO("Initializing HikvisionCamera Subscribers");
    ptz_controls_sub = pub_node.subscribe(PTZ_CONTROLS_TOPIC, ROS_RATE, &HikvisionCamera::ptzControlsCallback, this);
    ptz_key_teleop_controls_sub = pub_node.subscribe(PTZ_KEYBOARD_TELEOP_CONTROLS_TOPIC, 10, &HikvisionCamera::ptzKeyboardControlsCallback, this);
    ptz_joy_teleop_controls_sub = pub_node.subscribe(PTZ_JOYSTICK_TELEOP_CONTROLS_TOPIC, 10, &HikvisionCamera::ptzJoystickControlsCallback, this);

}


void HikvisionCamera::initializeROSPublishers()
{
    ROS_INFO("Initializing HikvisionCamera Publishers");
    // Commented out since video_stream_opencv ROS node will stream video
  /*  camera_info_mgr.reset(new camera_info_manager::CameraInfoManager(priv_node, camera_name, camera_info_url));
    if (!camera_info_mgr->isCalibrated())
    {
        camera_info_mgr->setCameraName(camera_name);
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.frame_id = frame_id;
        camera_info.width = (unsigned int)(image_width);
        camera_info.height = (unsigned int)(image_height);
        camera_info_mgr->setCameraInfo(camera_info);
    }
    image_transport::ImageTransport it(priv_node);
    image_pub = it.advertiseCamera(camera_name, 1);*/

}


void HikvisionCamera::initializeROSServices()
{
    ROS_INFO("Initializing HikvisionCamera Services");
    SetCameraInfoSrv = priv_node.advertiseService(camera_name+"/set_camera_info",&HikvisionCamera::setCameraInfo,this);
}


HikvisionCamera::HikvisionCamera(): priv_node("~"){
    ROS_INFO("in class constructor of HikvisionCamera");

    initializeHikvisionCameraAtributes();

    initializeROSParameters();
    initializeROSSubscribers();
    initializeROSPublishers();
    initializeROSServices();

    initHikSDK();
}

void HikvisionCamera::run()
{
    ros::Rate loop_rate(ROS_RATE);
    while (priv_node.ok()){
        ros::spin();
    }
}

HikvisionCamera::~HikvisionCamera()
{

    if(user_id)
    {
        NET_DVR_Logout_V30(user_id);
    }
    if(data_play_handler>0)
    {
        NET_DVR_StopRealPlay(data_play_handler);
    }

    NET_DVR_Cleanup();

    ROS_INFO("[%s] END",camera_name.c_str());
}
