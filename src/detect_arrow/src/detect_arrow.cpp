#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"

class Arrow_detector:public rclcpp::Node{
    public:
    Arrow_detector():Node("Arrow_detector"){
        using namespace std::placeholders;
        this->subscription_=this->create_subscription<sensor_msgs::msg::Image>(
            "OriginalVideo",
            10,
            std::bind(&Arrow_detector::MainArrowDetector,this,_1)
        );
        RCLCPP_INFO(this->get_logger(),"Arrow_detector created !");
    }
    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg);
};

void Arrow_detector::MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat originalframe=cv_ptr->image;
    RCLCPP_INFO(this->get_logger(), "Get frame");
    cv::imshow("Video",originalframe);
    cv::waitKey(33);
}


int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Arrow_detector>());
    rclcpp::shutdown();
}