#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include "interfaces/srv/imagerequest.hpp"
#include <filesystem>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class PictureDriver : public rclcpp::Node{
    public:
    PictureDriver():Node("PictureDriver"){
        publisher_=this->create_publisher<sensor_msgs::msg::Image>("OriginalVideo",10);
        this->declare_parameter<std::string>("PicturePath",std::string("/home/lqx/code/Engineering_robot_RM2025_Pnx/Pictures"));
    }
    void publish();
    private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

};

void PictureDriver::publish(){

    std::string Path=this->get_parameter("PicturePath").as_string();

    if(!(fs::exists(Path)&&fs::is_directory(Path))){
        RCLCPP_ERROR(this->get_logger(),"Picture path is not exist or not a directory");
        return;
    }

    for(const auto & entry: fs::directory_iterator(Path)){
        std::this_thread::sleep_for(33ms);
        if(entry.is_directory()) continue;
        if(entry.path().extension()!=".jpg") continue;
        cv::Mat frame=cv::imread(entry.path().string());
        if(frame.empty()){
            RCLCPP_ERROR(this->get_logger(),"Can not open picture");
            return;
        }
        else RCLCPP_INFO(this->get_logger(),"Open picture : %s",entry.path().string().c_str());

        auto image_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg();
        publisher_->publish(*image_ptr);
        RCLCPP_INFO(this->get_logger(),"Publish picture");
    }
}


int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    auto Picture_=std::make_shared<PictureDriver>();
    Picture_->publish();
    rclcpp::spin(Picture_);
    rclcpp::shutdown();
    return 0;
}