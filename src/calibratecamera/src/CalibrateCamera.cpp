#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> subscriber_;
int CHECKERBOARD[2];

std::vector<std::vector<cv::Point3f> > objpoints;
std::vector<std::vector<cv::Point2f> > imgpoints;


void InitCalibrationParam(){
    CHECKERBOARD[0]=7;
    CHECKERBOARD[1]=7;
}

void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    if(!rclcpp::ok()){
        rclcpp::shutdown();
    }
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    RCLCPP_INFO(node->get_logger(),"Get Image");
    cv::Mat originalimage=cv_ptr->image;
    cv::imshow("get",originalimage);
    cv::waitKey(22);
}

int main (int argc,char** argv){
    rclcpp::init(argc,argv);
    node=std::make_shared<rclcpp::Node>("calibrate_camera");
    subscriber_=node->create_subscription<sensor_msgs::msg::Image>("OriginalVideo",10,ImageCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
}