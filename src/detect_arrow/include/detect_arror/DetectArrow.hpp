#ifndef __DetectArrow__
#define __DetectArrow__

#include "interfaces/srv/imagerequest.hpp"
#include "interfaces/msg/redeem_box_position.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Filter.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define arrow_draw
#define Imageshow
// #define TargetArrowtest
#define twopath_inoneline
#define test_pcl_manage
#define noMainDetectArrow

using namespace std::chrono;
using namespace std::placeholders;

typedef std::pair<int,int> pii;

typedef cv::Vec2f LineAL;
typedef std::vector<LineAL> LineALs;
typedef LineAL Line;
typedef LineALs Lines;

//Ax+By+C=0
struct LineABC{
    double a,b,c;
};

typedef std::vector<std::vector<cv::Point>> Counters;
typedef std::vector<cv::Point> Counter;
typedef std::vector<std::vector<cv::Point2d>> Counter2ds;
typedef std::vector<cv::Point2d> Counter2d;

//normalize vector with a point on the line
typedef cv::Vec4d LineVP;

class Arrow_detector:public rclcpp::Node{
    public:
    // using Imagerequest=interfaces::srv::Imagerequest;
    Arrow_detector(double k);

    // static cv::Mat OOriginalImage;
    private:
    // rclcpp::Client<Imagerequest>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::RedeemBoxPosition>::SharedPtr RedeemBoxPosition_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr label_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // void MainArrowDetector(const sensor_msgs::msg::Image::SharedPtr msg);
    // rclcpp::Node::SharedPtr node_shred_ptr;
    cv::Mat OriginalImage_;
    cv::Mat GreyImage;
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
    std::vector<cv::Point2d> TargetArrow(const cv::Mat & BinaryImage,cv::Mat & Image);

    void DrawPnPResult(cv::Mat & Image, const cv::Mat & rvec, const cv::Mat & tvec, cv::Scalar color, int thickness, cv::Point textpos);

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
    double ArrowDetectorLongShortRateMax;
    double ArrowDetectorLongShortRateMin;

    // object params

    // 相机外参
    std::vector<double> cameraMatrix;
    // 相机外参 Eigen
    Eigen::Matrix<double,3,3> cameraMatrixEigen;
    // 相机外参 Eigen Inverse
    Eigen::Matrix<double,3,3> InverseCameraMatrixEigen;
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
    //frontface center of redemption 
    Eigen::Matrix<double,4,1> frontfacecenter;
    // 兑换框正面到箭头的变换矩阵
    // Eigen::Matrix<double,4,4> CenterToArrowvec;

    // 降维矩阵
    Eigen::Matrix<double,3,4> signMat;

    // box to camera
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_box_to_camera;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_camera_to_arm;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_camera_to_map;

    YAML::Node config;

public: //pcl manage
    template <typename PointT>
    pcl::PointCloud<PointT> PointCloudTransformer(const pcl::PointCloud<PointT>& inputcloud,
    std::string sourceframe,
    std::string targetframe);

private:
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> msgfillter_cloudpoint_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> msgfillter_image_sub;

    // set sync policy as Approximate
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    tf2_ros::Buffer::SharedPtr tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    //OriginalImage_ for plc manage
    cv::Mat OriginalImage_pcl;

    // Init Function for pointcloud part;
    void PointCloudeInit();

    void ImageCloudPointCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    
    template<typename T>
    void DrawCircleMask(cv::Mat Image,std::vector<cv::Point_<T>> counter);

    //in or on circle
    bool inCircle(const cv::Point2f & Center,
        const float & CornerPointsRadius,
        const Eigen::Matrix<double,3,1>& TestPoint);

    // idea1 get plant first and using the limit of plant to get the possition of corner points and then get the trvec
    bool GetTRvecPointCloud_PC(const pcl::PointCloud<pcl::PointXYZ> &pointcloud, 
        Counter2d CornerPoints, 
        cv::Mat & tvec, 
        cv::Mat & rvec);
    
    // kown a plant and know the 2D points are on that plant
    // get the 3D points according to these
    // @param Points2D 2D point on plant
    // @param plant Eigen::VectorXf plant Ax+By+Cz+D=0
    // @param Points3D 
    bool ImagePointTo3DPoint_Plant(const Counter2d& Points2D, const Eigen::VectorXf & plant, std::vector<cv::Point3d> &Points3D);


    # ifdef test_pcl_manage

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_test_point_cloud_pub;

    # endif

    double ransacDistanceThreshold;
    int ransacMaxIterations;

};


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

cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R);

// std::vector<cv::Point2d> subopix(const cv::Mat& GrayImage, std::vector<cv::Point> int_pointset, cv::Size winSize, cv::Size zeroZone, cv::TermCriteria criteria, cv::InputArray mask=cv::noArray());
void subopix(const cv::Mat& GrayImage, std::vector<cv::Point2d>& pointset, cv::Size winSize, cv::Size zeroZone, cv::TermCriteria criteria, cv::InputArray mask=cv::noArray());

template<typename T,typename G>
bool IsPointSame(cv::Point_<T> point1,cv::Point_<G> point2);


// @param plant plant equality is Ax+By+Cz+D=0
// @param coff two of X,Y,Z in order
// @param emptyplace UN need to get index begin with 0
double CalculatePlantEquality(Eigen::VectorXf plant,std::vector<double> coff,int emptyplace);

// KabschAlgorithm
bool KabschAlgorithm(const std::vector<cv::Point3d> &Source,
    const std::vector<cv::Point3d>& Target,
    cv::Mat & tvec,
    cv::Mat & rvec);

#endif