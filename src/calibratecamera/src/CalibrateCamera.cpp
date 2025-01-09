#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <mutex>
#include <fstream>
#include <sstream>

using namespace std::chrono;

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> subscriber_;
int CHECKERBOARD[2];

std::vector<std::vector<cv::Point3f> > objpoints;
std::vector<std::vector<cv::Point2f> > imgpoints;
std::vector<cv::Point3f> objp;
std::mutex mtx;

void InitCalibrationParam(){
    CHECKERBOARD[0]=10;//宽度
    CHECKERBOARD[1]=7;//高度
    for(int i=0;i<CHECKERBOARD[1];i++) for(int j=0;j<CHECKERBOARD[0];j++){
        objp.push_back(cv::Point3f(j,i,0));
    }
    node->declare_parameter<std::string>("PicturePath",std::string("/home/lqx/code/Engineering_robot_RM2025_Pnx/Pictures"));
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
    cv::Mat msgg=cv_ptr->image;
    cv::Mat originalimage;
    cv::cvtColor(msgg,originalimage,cv::COLOR_BGR2GRAY);
    RCLCPP_INFO(node->get_logger(),"Get Image");
    
    std::vector<cv::Point2f> corner_pts;
    bool success=0;
    success=cv::findChessboardCorners(originalimage,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),corner_pts);

    if(!success){
        RCLCPP_WARN(node->get_logger(),"fail to find circls grid");
        return;
    }
    else{
        RCLCPP_INFO(node->get_logger(),"find circls grid successfully");
        cv::cornerSubPix(originalimage,corner_pts,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
    }

    //

    cv::drawChessboardCorners(msgg, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

    // mtx.lock();
    objpoints.push_back(objp);
    imgpoints.push_back(corner_pts);

    cv::Mat cameraMatrix, distCoeffs, R, T;

    if(objpoints.size()>=20){
        // std::ofstream OUT;
        std::stringstream ss;
        // OUT.open(node->get_parameter("parampath").as_string());
        // if(OUT.is_open()){
        //     RCLCPP_ERROR(node->get_logger(),"open file fail");
        //     rclcpp::shutdown();
        // }
        // else RCLCPP_INFO(node->get_logger(),"open file succesfully");
        cv::calibrateCamera(objpoints,imgpoints,cv::Size(originalimage.rows,originalimage.cols),cameraMatrix,distCoeffs,R,T);
        ss << "cameraMatrix : " << cameraMatrix << std::endl;
        // 透镜畸变系数
        ss << "distCoeffs : " << distCoeffs << std::endl;
        // rvecs
        ss<< "Rotation vector : " << R << std::endl;
        // tvecs
        ss << "Translation vector : " << T << std::endl;
        RCLCPP_INFO(node->get_logger(),"finish!");
        RCLCPP_INFO(node->get_logger(),"%s",ss.str().c_str());
        objpoints.clear();
        imgpoints.clear();
        // OUT.close();
        rclcpp::shutdown();
    }
    else{
        RCLCPP_INFO(node->get_logger(),"get %ld frames",objpoints.size());
        cv::imshow("get",msgg);
        cv::imwrite(node->get_parameter("PicturePath").as_string()+"/"+std::to_string(objpoints.size())+".jpg",originalimage);
        // cv::waitKey(1500);
        // std::this_thread::sleep_for(500ms);
    }

    // mtx.unlock();

}

int main (int argc,char** argv){
    rclcpp::init(argc,argv);
    node=std::make_shared<rclcpp::Node>("calibrate_camera");
    InitCalibrationParam();
    node->declare_parameter<std::string>("parampath","");
    subscriber_=node->create_subscription<sensor_msgs::msg::Image>("OriginalVideo",10,ImageCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
}