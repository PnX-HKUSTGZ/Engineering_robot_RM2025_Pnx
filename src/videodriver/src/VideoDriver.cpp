#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
// #include <bits/stdc++.h>

namespace ENGINEER_RM_25{

template <typename... Args>
void PrintINFO(const rclcpp::Logger & logger,const Args&... args){
    std::stringstream ss;
    (ss<<...<<args);//C++17 展开参数，括号不能去除
    RCLCPP_INFO(logger,ss.str().c_str());
}

class VideoDriver:public rclcpp::Node{
    public:
    VideoDriver():Node("VideoDriver"){
        using namespace std::chrono;
        publisher_=this->create_publisher<sensor_msgs::msg::Image>("OriginalVideo",10);
        timer_=this->create_wall_timer(33ms,std::bind(&VideoDriver::PublishVideoCallBack,this));
        this->declare_parameter<std::string>("VideoPath",std::string("/home/lqx/code/Engineering_robot_RM2025_Pnx/video/Video_20241228180203703.mp4"));
        PrintINFO(this->get_logger(),"VideoDriver is running");
    }
    private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video;
    void PublishVideoCallBack();
};

void VideoDriver::PublishVideoCallBack(){
    video.open(this->get_parameter("VideoPath").as_string().c_str());
    PrintINFO(this->get_logger(),"Open video in",this->get_parameter("VideoPath").as_string());
    if(!video.isOpened()){
        timer_->cancel();
        PrintINFO(this->get_logger(),"Video is not opened");
        rclcpp::shutdown();
        return;
    }
    cv::Mat frame;
    video>>frame;
    if(frame.empty()){
        timer_->cancel();
        PrintINFO(this->get_logger(),"Video comes to end");
        rclcpp::shutdown();
        return;
    }
    cv::imshow("???",frame);
    cv::waitKey(33);
    publisher_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg());
    PrintINFO(this->get_logger(),"Publish video");
}

}

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ENGINEER_RM_25::VideoDriver>());
    rclcpp::shutdown();
    return 0;
}