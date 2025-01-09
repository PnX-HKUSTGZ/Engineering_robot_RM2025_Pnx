#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/srv/imagerequest.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <thread>
#include <algorithm>
#include <sstream>
#include <Eigen/Dense>

// #ifdef __DetectArrow__
// #define __DetectArrow__

// #define DeBug
#define DeBugHough

using namespace std::chrono;
using namespace std::placeholders;

std::vector<double> cameraMatrix={2375.787121776882, 0, 740.0689192411256,
 0, 2379.743671056914, 590.1717549829305,
 0, 0, 1};

std::vector<double> distCoeffs={-0.07093428455159315, 0.1865900206019591, 0.003095286426499801, 0.004747807496693957, 0.8787773017757813};

std::vector<cv::Point3d> objpoints={cv::Point3d(0,0,0),cv::Point3d(10,10,0),cv::Point3d(136.42135623730950488016887242097,5,0),cv::Point3d(5,136.42135623730950488016887242097,0)};

std::vector<Eigen::Matrix<double,4,1>> objpointsEigen{
    Eigen::Matrix<double,4,1>(0,0,0,1),
    Eigen::Matrix<double,4,1>(10,10,0,1),
    Eigen::Matrix<double,4,1>(136.42135623730950488016887242097,5,0,1),
    Eigen::Matrix<double,4,1>(5,136.42135623730950488016887242097,0,1)
};

std::vector<cv::Point3d> ObjRedemptionBoxCornerPoint={
    cv::Point3d(-117.02617228637361528833974192835,52.679455198397790567862904976811,24),
    cv::Point3d(52.679455198397790567862904976811,-117.02617228637361528833974192835,24),
    cv::Point3d(-117.02617228637361528833974192835,52.679455198397790567862904976811,264),
    cv::Point3d(52.679455198397790567862904976811,-117.02617228637361528833974192835,264)
};

class Arrow_detector:public rclcpp::Node{
    public:
    using Imagerequest=interfaces::srv::Imagerequest;
    Arrow_detector():Node("Arrow_detector"){
        this->client_=this->create_client<Imagerequest>("OriginalVideo");
        this->subscription_=this->create_subscription<sensor_msgs::msg::Image>("OriginalVideo",10,std::bind(&Arrow_detector::GetImage,this,_1));
        RCLCPP_INFO(this->get_logger(),"Arrow_detector client created !");
    }
    void CreatGetImageTimer();
    void GetImage(const sensor_msgs::msg::Image::SharedPtr msg);
    private:
    rclcpp::Client<Imagerequest>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    // void MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Node::SharedPtr node_shred_ptr;
    cv::Mat PreProgress(const cv::Mat & OriginalImage);
    bool MainDetectArrow(const cv::Mat & OriginalImage);
    bool TargetArrow(const cv::Mat & BinaryImage);
    cv::Mat OriginalImage;
    std::vector<cv::Point2d> ArrowPeaks;
    bool PnPsolver();
    std::vector<cv::Point2i> ImageRedemptionBoxCornerPoints;
    cv::Mat rvec,tvec;
    // cv::VideoWriter ddd("/home/lqx/code/Engineering_robot_RM2025_Pnx/video.mp4",cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0,cv::Size(1440,1080));
    cv::VideoWriter videowriter=cv::VideoWriter("/home/lqx/code/Engineering_robot_RM2025_Pnx/video.avi",cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),30.0,cv::Size(1440,1080));
};

typedef cv::Vec2f Line;
typedef std::vector<Line> Lines;

void DrawLines(cv::Mat & img,const Lines & lines, const cv::Scalar& color,
    int thickness = 1, int lineType = cv::LINE_8, int shift = 0){
    for(const auto & i : lines){
        float rho=i[0],theta=i[1];
        double a=std::cos(theta),b=std::sin(theta);
        double x0=a*rho,y0=b*rho;
        cv::Point pt1(cvRound(x0+9000*(-b)),cvRound(y0+9000*(a)));
        cv::Point pt2(cvRound(x0-9000*(-b)),cvRound(y0-9000*(a)));
        cv::line(img,pt1,pt2,color,thickness,lineType,shift);
    }
}

double DistanceBetweenPointAndLine(const cv::Point2f & p,const Line & l){
    double r=l.val[0],theta=l.val[1];
    double t1=(p.y*std::cos(theta)-p.x*std::sin(theta));
    double t0=(r-p.x*std::cos(theta)-p.y*std::sin(theta));
    cv::Point2f p1(r*std::cos(theta)-t1*sin(theta),r*std::sin(theta)+t1*cos(theta));
    cv::Point2f p2(p.x+t0*cos(theta),p.y+t0*sin(theta));
    if(p1!=p2){
        RCLCPP_ERROR(rclcpp::get_logger("DistanceBetweenPointAndLine"),"p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%lf,%lf]",p1.x,p1.y,p2.x,p2.y,p.x,p.y);
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("DistanceBetweenPointAndLine"),"r : [%lf],theta : [%lf] ,p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%lf,%lf], distance: [%lf]",r,theta,p1.x,p1.y,p2.x,p2.y,p.x,p.y,cv::norm(p-p1));
    return cv::norm(p-p1);
}

double DistanceBetweenPointAndLine(const cv::Point & p,const Line & l){
    double r=l.val[0],theta=l.val[1];
    double t1=(p.y*std::cos(theta)-p.x*std::sin(theta));
    double t0=(r-p.x*std::cos(theta)-p.y*std::sin(theta));
    cv::Point2f p1(r*std::cos(theta)-t1*sin(theta),r*std::sin(theta)+t1*cos(theta));
    cv::Point2f p2(p.x+t0*cos(theta),p.y+t0*sin(theta));
    if(p1!=p2){
        RCLCPP_ERROR(rclcpp::get_logger("DistanceBetweenPointAndLine"),"p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%d,%d]",p1.x,p1.y,p2.x,p2.y,p.x,p.y);
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("DistanceBetweenPointAndLine"),"r : [%lf],theta : [%lf] ,p1 : [%lf,%lf],p2 : [%lf,%lf],p : [%d,%d], distance: [%lf]",r,theta,p1.x,p1.y,p2.x,p2.y,p.x,p.y,cv::norm(cv::Point2f(p.x,p.y)-p1));
    return cv::norm(cv::Point2f(p.x,p.y)-p1);
}

// #endif