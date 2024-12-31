#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include "interfaces/srv/imagerequest.hpp"
// #include <bits/stdc++.h>


namespace ENGINEER_RM_25{

std::mutex mtxvideo;

template <typename... Args>
void PrintINFO(const rclcpp::Logger & logger,const Args&... args){
    std::stringstream ss;
    (ss<<...<<args);//C++17 展开参数，括号不能去除
    RCLCPP_INFO(logger,ss.str().c_str());
}

class VideoDriver:public rclcpp::Node{
    public:
    using Imagerequest=interfaces::srv::Imagerequest;
    VideoDriver():Node("VideoDriver"){
        using namespace std::chrono;
        using namespace std::placeholders;
        service_=this->create_service<Imagerequest>("OriginalVideo",std::bind(&VideoDriver::PublishVideoCallBack,this,_1,_2));
        this->declare_parameter<std::string>("VideoPath",std::string("/home/lqx/code/Engineering_robot_RM2025_Pnx/video/Video_20241228180155626.avi"));
        PrintINFO(this->get_logger(),"VideoDriver service is running");

        video.open(this->get_parameter("VideoPath").as_string().c_str());
        PrintINFO(this->get_logger(),"Open video in",this->get_parameter("VideoPath").as_string());
        RCLCPP_INFO(this->get_logger(),"Waiting.....");
    }
    private:
    rclcpp::Service<Imagerequest>::SharedPtr service_;
    // rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video;
    void PublishVideoCallBack(const std::shared_ptr<Imagerequest::Request> req,std::shared_ptr<Imagerequest::Response> res);
};

void VideoDriver::PublishVideoCallBack(const std::shared_ptr<Imagerequest::Request>req,std::shared_ptr<Imagerequest::Response> res){
    (void)req;
    cv::Mat frame;
    mtxvideo.lock();
    video>>frame;
    mtxvideo.unlock();
    if(frame.empty()){
        res->end=1;
        PrintINFO(this->get_logger(),"Video comes to end");
        rclcpp::shutdown();
        return;
    }
    res->end=0;
    // cv::imshow("???",frame);
    // cv::waitKey(3);
    RCLCPP_INFO(this->get_logger(),"SIZE of fram : [%d,%d]",(int)(frame.size().height),(frame.size().width));

    auto imageptr=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg();
    res->image=*imageptr;
    PrintINFO(this->get_logger(),"Publish video");
}

}

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ENGINEER_RM_25::VideoDriver>());
    rclcpp::shutdown();
    return 0;
}