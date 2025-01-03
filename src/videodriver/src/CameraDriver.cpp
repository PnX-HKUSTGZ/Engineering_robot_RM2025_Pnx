#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include "interfaces/srv/imagerequest.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "VideoDriver/MvCameraControl.h"

using namespace std::chrono;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher_;
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Service<interfaces::srv::Imagerequest>> service_;
int nRet = MV_OK;


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        RCLCPP_INFO(node->get_logger(),"The Pointer of pstMVDevInfo is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        RCLCPP_INFO(node->get_logger(),"Device Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        RCLCPP_INFO(node->get_logger(),"CurrentIp: %d.%d.%d.%d" , nIp1, nIp2, nIp3, nIp4);
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        RCLCPP_INFO(node->get_logger(),"Device Model Name: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        RCLCPP_INFO(node->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        RCLCPP_INFO(node->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        RCLCPP_INFO(node->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        RCLCPP_INFO(node->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        RCLCPP_INFO(node->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),"Not support.");
    }

    return true;
}

void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser){
    if(!rclcpp::ok()){
        rclcpp::shutdown();
    }
    if (!pFrameInfo) return;
    (void)pUser;
    RCLCPP_INFO(node->get_logger(),"GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]", 
        pFrameInfo->nExtendWidth, pFrameInfo->nExtendHeight, pFrameInfo->nFrameNum);

    cv::Mat OriginalImage(pFrameInfo->nExtendHeight, pFrameInfo->nExtendWidth,CV_8UC1,pData);
    cv::Mat imageRGB;
    cv::cvtColor(OriginalImage, imageRGB, cv::COLOR_BayerRG2RGB);

    auto image_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",imageRGB).toImageMsg();

    publisher_->publish(*image_ptr);
    cv::imshow("Camera",imageRGB);
    // cv::imshow("Camera1",OriginalImage);
    cv::waitKey(22);
    RCLCPP_INFO(node->get_logger(),"publish video");
}

void GainAdjustment(void* handle){
    //load in
    int ExposureTimeLower=node->get_parameter("ExposureTimeLower").as_int();
    int ExposureTimeUpper=node->get_parameter("ExposureTimeUpper").as_int();

    MV_CC_SetGain(handle,node->get_parameter("Gain").as_int());

    MVCC_FLOATVALUE exposurtime;
    nRet=MV_CC_GetExposureTime(handle,&exposurtime);
    if(nRet!=MV_OK){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_GetExposureTime fail! nRet [%x]",nRet);
    }
    else{
        RCLCPP_INFO(node->get_logger(),"FPS : %f",exposurtime.fCurValue);
        RCLCPP_INFO(node->get_logger(),"Upper exposure time : %f",exposurtime.fMax);
        RCLCPP_INFO(node->get_logger(),"Lower exposure time : %f",exposurtime.fMin);
    }

    nRet=MV_CC_SetAutoExposureTimeUpper(handle,ExposureTimeUpper);
    if(nRet!=MV_OK){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_SetAutoExposureTimeUpper fail! nRet [%x]",nRet);
    }
    else RCLCPP_INFO(node->get_logger(),"Set ExposureTimeUpper : %d",ExposureTimeUpper);

    nRet=MV_CC_SetAutoExposureTimeLower(handle,ExposureTimeLower);
    if(nRet!=MV_OK){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_SetAutoExposureTimeLower fail! nRet [%x]",nRet);
    }
    else RCLCPP_INFO(node->get_logger(),"Set ExposureTimeLower : %d",ExposureTimeLower);


}


void publish_video(){
    void* handle = NULL;
    nRet = MV_CC_Initialize();
do{
    if (MV_OK != nRet){
        RCLCPP_INFO(node->get_logger(),"Initialize SDK fail! nRet [0x%x]", nRet);
        break;
    }
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_EnumDevices fail! nRet [%x]", nRet);
        break;
    }
    if (stDeviceList.nDeviceNum > 0){
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++){
            RCLCPP_INFO(node->get_logger(),"[device %d]:", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo){
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }
    } 
    else{
        RCLCPP_ERROR(node->get_logger(),"Find No Devices!");
        break;
    }

    RCLCPP_INFO(node->get_logger(),"Please Intput camera index: ");
    unsigned int nIndex = 0;
    // scanf("%d", &nIndex);

    if (nIndex >= stDeviceList.nDeviceNum){
        RCLCPP_ERROR(node->get_logger(),"Intput error!");
        break;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_CreateHandle fail! nRet [%x]", nRet);
        break;
    }
    else RCLCPP_INFO(node->get_logger(),"MV_CC_CreateHandle correct");

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_OpenDevice fail! nRet [%x]", nRet);
        break;
    }
    else RCLCPP_INFO(node->get_logger(),"MV_CC_OpenDevice successfully");

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
        break;
    }

    // 注册抓图回调
    // register image callback
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_RegisterImageCallBackEx fail! nRet [%x]", nRet);
        break; 
    }
    else RCLCPP_INFO(node->get_logger(),"MV_CC_RegisterImageCallBackEx correct");

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_StartGrabbing fail! nRet [%x]", nRet);
        break;
    }
    else RCLCPP_INFO(node->get_logger(),"MV_CC_StartGrabbing correct");

    GainAdjustment(handle);
    std::this_thread::sleep_for(100000ms);

    // 停止取流
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_StopGrabbing fail! nRet [%x]", nRet);
        break;
    }
    else{
        RCLCPP_INFO(node->get_logger(),"MV_CC_StopGrabbing correct");
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        RCLCPP_ERROR(node->get_logger(),"MV_CC_CloseDevice fail! nRet [%x]", nRet);
        break;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet){
        RCLCPP_ERROR(node->get_logger(),"MV_CC_DestroyHandle fail! nRet [%x]", nRet);
        break;
    }
    handle = NULL;
}while(0);

    if(handle!=NULL){
        MV_CC_DestroyHandle(handle);
        handle=NULL;
    }
    MV_CC_Finalize();
}

int main (int argc,char ** argv){
    rclcpp::init(argc,argv);
    node=std::make_shared<rclcpp::Node>("CameraDriver");
    node->declare_parameter<int>("ExposureTimeLower",70000);
    node->declare_parameter<int>("ExposureTimeUpper",70000);
    node->declare_parameter<int>("Gain",15);
    node->declare_parameter<int>("VideoDriverModle",1);
    // std::shared_ptr<rclcpp::TimerBase> timer_=node->create_wall_timer(22ms,publish_video);
    if(node->get_parameter("VideoDriverModle").as_int()==1){
        publisher_=node->create_publisher<sensor_msgs::msg::Image>("OriginalVideo",10);
        while(rclcpp::ok()) publish_video();
    }
    rclcpp::spin(node);
}