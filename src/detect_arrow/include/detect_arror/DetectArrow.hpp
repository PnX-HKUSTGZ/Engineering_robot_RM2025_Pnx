#ifndef __DetectArrowS__
#define __DetectArrow__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/srv/imagerequest.hpp"
#include "interfaces/msg/redeem_box_position.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Filter.hpp"
#include <thread>
#include <algorithm>
#include <sstream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#define arrow_draw

using namespace std::chrono;
using namespace std::placeholders;

class Arrow_detector:public rclcpp::Node{
    public:
    // using Imagerequest=interfaces::srv::Imagerequest;
    Arrow_detector(double k);

    // static cv::Mat OOriginalImage;
    private:
    // rclcpp::Client<Imagerequest>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::RedeemBoxPosition>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr label_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // void MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg);
    // rclcpp::Node::SharedPtr node_shred_ptr;
    cv::Mat OriginalImage_;
    std::vector<cv::Point2d> ArrowPeaks;
    std::vector<cv::Point2i> ImageRedemptionBoxCornerPoints;
    cv::Mat rvec,tvec;
    FilterCorner filter_;

    cv::Mat PreProgress(const cv::Mat & OriginalImage);
    // void InitialArrowDetector();
    void GetImage(const sensor_msgs::msg::Image::SharedPtr msg);
    bool PnPsolver(const std::vector<cv::Point2d > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
        cv::Mat & rvec, cv::Mat & tvec, bool useExtrinsicGuess, int flags);
    bool MainDetectArrow(const cv::Mat & OriginalImage);
    bool TargetArrow(const cv::Mat & BinaryImage);

    // cv::VideoWriter ddd("/home/lqx/code/Engineering_robot_RM2025_Pnx/video.mp4",cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0,cv::Size(1440,1080));
    // cv::VideoWriter videowriter=cv::VideoWriter("/home/lqx/code/Engineering_robot_RM2025_Pnx/video.avi",cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),30.0,cv::Size(1440,1080));

    // parameters setting

    // detect params
    int ArrowDetectorPixelNumMax;
    int ArrowDetectorPixelNumMin;
    double ArrowDetectorLengthWidthRatioMax;
    double ArrowDetectorLengthWidthRatioMin;
    int ArrowDetectorApproxSizeMax;
    int ArrowDetectorApproxSizeMin;
    double ArrowDetectorCannyThreshold1;
    double ArrowDetectorCannyThreshold2;
    double ArrowDetectorHoughRho;
    double ArrowDetectorHoughTheta;
    int ArrowDetectorHoughThreshold;
    double ArrowDetectParallelThreshold;
    double ArrowDetectorThresholdThresh;
    double ArrowDetectorThresholdMaxval;
    double ArrowDetectorThresholdThreshold;
    int ArrowDetectorIterations;
    double ArrowDetectorapproxPolyDPEpsilon;

    // object params

    // 相机外参
    std::vector<double> cameraMatrix;
    // 相机外参 Eigen
    Eigen::Matrix<double,3,3> cameraMatrixEigen;
    // 相机内参
    std::vector<double> distCoeffs;
    // 箭头上的点
    std::vector<cv::Point3d> objpoints;
    // 箭头上的点  Eigen
    std::vector<Eigen::Matrix<double,4,1>> objpointsEigen;
    //  redemption box 上的点 
    std::vector<cv::Point3d> ObjRedemptionBoxCornerPoint;
    //  redemption box 上的点  Eigen
    std::vector<Eigen::Matrix<double,4,1>> ObjRedemptionBoxCornerPointEigen;
    // 箭头旁的直线
    std::vector<Eigen::Matrix<double,4,1>> Object2cornersEigen;
    // 兑换框正面到箭头的变换矩阵
    Eigen::Matrix<double,4,4> CenterToArrowvec;
    // 降维矩阵
    Eigen::Matrix<double,3,4> signMat;
};

typedef std::pair<int,int> pii;

typedef cv::Vec2f LineAL;
typedef std::vector<LineAL> LineALs;
typedef LineAL Line;
typedef LineALs Lines;

//Ax+By+C=0
struct LineABC{
    double a,b,c;
};

//normalize vector with a point on the line
typedef cv::Vec4d LineVP;

LineAL GetLineAL(const LineVP & l);
LineAL GetLineAL(const LineABC & l);

LineABC GetLineABC(const Line & l);
LineABC GetLineABC(const LineVP & l);

void DrawLines(cv::Mat & img,const Lines & lines, const cv::Scalar& color,
    int thickness = 1, int lineType = cv::LINE_8, int shift = 0);

void DrawLines(cv::Mat & img,const std::vector<LineVP> & lines, const cv::Scalar& color,
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

template<typename T,typename G>
double DistancePoints(const cv::Point_<T> & p1,const cv::Point_<G> & p2);

// 发现不在给定的多边形上,就返回0
template<typename T>
bool FindContinuePart(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point_<T> > & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold=5);

template<typename T>
bool FindContinuePart(const cv::Mat & BinaryImage,std::vector<cv::Point> & Pointset,const cv::Point & StartPoint,const std::vector<cv::Point_<T> > & Peaks,std::map<std::pair<int,int>,bool> &vis,const double PeaksThreshold,std::pair<cv::Point_<T>,cv::Point_<T> > & endpoints);

template<typename T>
void FindPolygonCounterPointsSets(const cv::Mat & BinaryImage,std::vector<std::vector<cv::Point>> & Pointssets,const std::vector<cv::Point_<T>> & Peaks,const double PeaksThreshold,std::vector<std::pair<cv::Point_<T>,cv::Point_<T>> > & Endpoints);

template<typename T>
void GetLinesIntersections(const std::vector<LineVP> & lines,std::vector<cv::Point_<T> > & Intersections);

cv::Point2f GetLineIntersections(const LineVP & line1,const LineVP & line2);

bool operator < (const cv::Point & a,const cv::Point & b);

#endif