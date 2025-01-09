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

class Arrow_detector:public rclcpp::Node{
    public:
    using Imagerequest=interfaces::srv::Imagerequest;
    Arrow_detector():Node("Arrow_detector"){
        this->client_=this->create_client<Imagerequest>("OriginalVideo");
        this->subscription_=this->create_subscription<sensor_msgs::msg::Image>("OriginalVideo",10,std::bind(&Arrow_detector::GetImage,this,_1));
        RCLCPP_INFO(this->get_logger(),"Arrow_detector client created !");
    }
    void InitialArrowDetector();
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
    int thickness = 1, int lineType = cv::LINE_8, int shift = 0);

double DistanceBetweenPointAndLine(const cv::Point2f & p,const Line & l);
double DistanceBetweenPointAndLine(const cv::Point & p,const Line & l);

const long double eps=1e-9;

struct Slope{
    int p1,p2;
    double slope;
};

typedef std::vector<std::vector<cv::Point>> Counters;
typedef std::vector<cv::Point> Counter;

double GetAngleAccordingToHorizon(cv::Point p1,cv::Point p2);

double GetAngle(cv::Point2f p1,cv::Point2f p2,cv::Point2f p3);
double DistancePoints(const cv::Point & p1,const cv::Point & p2);
double DistancePoints(const cv::Point2f & p1,const cv::Point & p2);
double DistancePoints(const cv::Point & p1,const cv::Point2f & p2);

// #endif