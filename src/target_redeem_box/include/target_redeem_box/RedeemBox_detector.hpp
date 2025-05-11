#ifndef __DetectArrow__
#define __DetectArrow__

#include "interfaces/srv/imagerequest.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "Filter.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #define arrow_draw
// #define Imageshow
// #define TargetArrowtest
#define test_pcl_manage
// #define test_pointcloud_main_log
// #define test_LocalCornerOpitimize
// #define drawFinalres
// #define DetectorRectangle_test
// #define DetectorRectangle_test_target

#define twopath_inoneline

using namespace std::chrono;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace Engineering_robot_RM2025_Pnx{

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
typedef std::vector<std::vector<cv::Point2f>> Counter2fs;
typedef std::vector<cv::Point2f> Counter2f;

//normalize vector with a point on the line
typedef cv::Vec4d LineVP;

struct PnPresult{
    cv::Mat tvec;
    cv::Mat rvec;
    rclcpp::Time stamp;
    PnPresult(const cv::Mat &tvec_,const cv::Mat &rvec_,rclcpp::Time tim=rclcpp::Clock().now());
};

template<typename T>
struct Circle{
    cv::Point_<T> center;
    float radius;
};

class CombGenerator{
    public:
    CombGenerator(int n_,int m_);
    std::vector<int> get_next();
    private:
    //length of the whole list
    //list look like :[0,1,2,...,n-1]
    int n;
    //num of places to choose
    int m;
    //record the place
    std::vector<int> recorder;
    bool update();
};

struct PlaneData {
    pcl::ModelCoefficients coefficients; // 平面模型系数 (a, b, c, d for ax + by + cz + d = 0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr points; // Point cloud containing only points belonging to this plane
};

struct RoiPointInfo {
    pcl::PointXYZ point3D;
    cv::Point2d point2D_proj;
    int roi_cloud_index; // Index in the PreprocessedCloudPoint cloud
};

class RedeemBox_detector:public rclcpp::Node{
    public:
    RedeemBox_detector(rclcpp::NodeOptions options=rclcpp::NodeOptions());

    private:

    std::string CloudPointTopic;
    std::string ImageTopic;
    std::string CloudPointFrame;
    std::string ImageFrame;

    // rclcpp::Client<interfaces::srv::Imagerequest>::SharedPtr image_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr label_image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    tf2_ros::Buffer::SharedPtr tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // mid360 Cloud
    pcl::PointCloud<pcl::PointXYZ> InTimeCloud;
    // mtx for InTimeCloud and CloudTimeStamp
    std::mutex Cloudmtx;

    // cloud queue
    std::queue<std::pair<int,rclcpp::Time>> CloudTimeStamp;
    // cloud queue buffertime
    rclcpp::Duration buffertime=rclcpp::Duration(0,0);

    std::vector<cv::Point2d> ArrowPeaks;
    std::vector<cv::Point2i> ImageRedemptionBoxCornerPoints;
    // FilterCorner filter_;

    // OriginalImage For main image
    cv::Mat OriginalImage;

    // for OriginalImage_ to store intime Image
    std::mutex OriginalImage_mutex;

    // for Image_sub_ to store intime Image
    void GetImage(const sensor_msgs::msg::Image::SharedPtr msg);

    bool PnPsolver(const std::vector<cv::Point2d > & ImagePoints2D,const std::vector<cv::Point3d > & ObjectPoints3D,const std::vector<double> & cameraMatrix,const std::vector<double> & distCoeffs,
        cv::Mat & rvec, cv::Mat & tvec, bool useExtrinsicGuess, int flags);

    void DrawPnPResult(cv::Mat & Image, const cv::Mat & rvec, const cv::Mat & tvec, cv::Scalar color, int thickness, cv::Point textpos);

    // send box position in camera to tf2
    // @param tvec translate vec
    // @param rvecmat rotate vec(3*1,1*3) or rotate mat(3*3)
    void SendBoxPosition(cv::Mat & tvec,cv::Mat & rvecmat,bool reverse=false, std::string frame_id="object/box");
    
    void CallDetectorFunctions();

    // this function will get mtx Cloudmtx
    void CloudSubManage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

    // InTimeCloudUpdate for InTime to clear some pass point cloud
    // this function will get mtx Cloudmtx
    void InTimeCloudUpdate();

    std::vector<std::function<int(const cv::Mat&)> > callback_functions;
    std::vector<std::string> callback_functions_names;


    std::thread ImageProcessorThread;

    // object params

    // 相机外参
    std::vector<double> cameraMatrix;
    // 相机外参 Mat
    cv::Mat cameraMatrixMat;
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

    // 相机得到的画面宽度
    int ImageWidth;
    // 相机得到的画面高度
    int ImageHeight;

    // 降维矩阵
    Eigen::Matrix<double,3,4> signMat;

private: // arrow detect

    void DetectArrowInit();

    cv::Mat PreProgress(const cv::Mat & OriginalImage);

    std::vector<cv::Point2d> TargetArrow(const cv::Mat & BinaryImage,cv::Mat & Image);

    int MainDetectArrow(const cv::Mat & OriginalImage);

    cv::Mat OriginalImage_ArrowDetect;

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

    // box to camera
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_box_to_camera;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_camera_to_arm;


    YAML::Node config;

public: //pcl manage
    template <typename PointT>
    pcl::PointCloud<PointT> PointCloudTransformer(const pcl::PointCloud<PointT>& inputcloud,
    std::string sourceframe,
    std::string targetframe);

private:

    //OriginalImage_ for plc manage
    cv::Mat OriginalImage_pcl;

    // Init Function for pointcloud part;
    void PointCloudeInit();

    // this function will get mtx Cloudmtx
    int MainPclManager(const cv::Mat& OriginalImage);
    
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
        cv::Mat & rvec,
        PlaneData & choosedPlane
        );
    
    // kown a plant and know the 2D points are on that plant
    // get the 3D points according to these
    // @param Points2D 2D point on plant
    // @param plant Eigen::VectorXf plant Ax+By+Cz+D=0
    // @param Points3D 
    bool ImagePointTo3DPoint_Plant(const Counter2d& Points2D, const Eigen::VectorXf & plant, std::vector<cv::Point3d> &Points3D);


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_test_point_cloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_camera_point_cloud_sub;
    # ifdef test_pcl_manage

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_camera_point_cloud_pub;

    # endif

    double ransacDistanceThreshold;
    int ransacMaxIterations;
    int ransacMinInliersNum;
    int ransacMaxPlaneNum;
    int PlaneOnCornersNumThreshold;
    double minPlaneDisThreshold;
    double CloseThresehold;
    double pnpPlaneCloseThresehold;
    double pnpPlaneCloseAfterThresehold;

private://Rectangle_Detector
    /*
    order of return vector
    @return if fall to detect well return an empty vector
    @return if success, will begin with special corner and continue in clock order
    */
    std::vector<cv::Point2f> TargetRectangle(const cv::Mat & BinaryImage,cv::Mat & Image);

    //using adaptthreshold instead of threshold
    cv::Mat Adapted_PreProgress(const cv::Mat & OriginalImage);
    
    int MainDetectArrow_Rectangle(const cv::Mat & OriginalImage);

    void RectangleDetectorInit();

    // @return 0 good 1 bad
    bool GetFourCornersPair(const Counters & Corners,Counters &OutputCorners);


    // OriginalImage of MainDetectArrow_Rectangle
    cv::Mat OriginalImage_Rectangle;

    double DetectRectangleMaxValue;
    // must be odd from 3 to 21
    int DetectRectangleBlockSize;
    double DetectRectangleC;
    int TargetRectangleapproxPolyDPepsilon;
    int TargetRectanglePixelNumMax;
    int TargetRectanglePixelNumMin;
    int TargetRectangleApproxSizeMax;
    int TargetRectangleApproxSizeMin;
    double TargetRectangleHWRateMax;
    double TargetRectangleHWRateMin;
    int TargetRectangleErodeItrations;
    int TargetRectangleDilateItrations;
    cv::Size TargetRectangleErodeCoreSize;
    cv::Size TargetRectangleDilateCoreSize;
    double TargetRectanglee4f2rateMin;
    double TargetRectanglee4f2rateMax;
    //alpha I + beta
    double DetectRectangleAlpha;
    //alpha I + beta
    double DetectRectangleBeta;

    double TargetRectangleSlopeVarianceThreshold;
    double TargetRectangleSlopeHorizonThreshold;

    double TargetRectanglePairFourCornersThreshold;

    std::vector<cv::Point3d> RectangleOuterlayerCorners;
    std::vector<Eigen::Matrix<double,4,1>> RectangleOuterlayerCornersEigen;
    std::vector<cv::Point3d> RectangleInnerlayerCorners;
    std::vector<Eigen::Matrix<double,4,1>> RectangleInnerlayerCornersEigen;



private: //同步结果并且发布
    std::queue<PnPresult> pnpress;
    std::queue<PnPresult> cloudress;
    std::shared_ptr<rclcpp::TimerBase> respubtimer_;
    int queuesiz;
    rclcpp::Duration syncThresehold=rclcpp::Duration(0,0);
    std::mutex pnpressMtx;
    std::mutex cloudressMtx;

    void SyncPubBoxPos();
    void SyncPubBoxPosInit();


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

template<typename T,typename G>
cv::Point_<double> Point3to2Transform(const Eigen::Matrix<G,3,3> & cameraMatrixEigen,const cv::Point3_<T> &point);

template<typename T,typename G>
std::vector<cv::Point_<double>> Points3to2Transform(const Eigen::Matrix<G,3,3> & cameraMatrixEigen,const std::vector<cv::Point3_<T>> &points);

void DrawRotatedRect(cv::Mat &Image,const cv::RotatedRect& rect, const cv::Scalar &color, int thinkness=1);

bool isPointInsideRotatedRect(const cv::Point2f& pt, const cv::RotatedRect& rect);


template<typename T>
cv::Point_<T> LocalCornerOpitimize(const cv::Mat & BinaryImage ,cv::Point_<T> Corner, int MaskRadius, int blockSize, int ksize, double k);

template<typename T>
void CombineCounters(const std::vector<std::vector<cv::Point_<T>>> & counters,std::vector<cv::Point_<T>> & output);

// append source to target
template<typename T>
void AppendCounters(const std::vector<std::vector<cv::Point_<T>>> & source,std::vector<cv::Point_<T>> & target);

void DrawTrangle(cv::Mat & Image,Counter2f & CornerPoints,cv::Scalar color,int thickness);

void DrawRect(cv::Mat & Image, const cv::Rect & rect, cv::Scalar color, int thickness);

/**
 * @brief 将点云分割成多个平面，返回平面模型系数和属于该平面的点云
 *
 * 此函数迭代地使用 RANSAC 查找平面，并将找到的平面点从点云中移除，在剩余点中继续查找。
 *
 * @param original_cloud 原始输入点云
 * @param distanceThreshold RANSAC分割的距离阈值
 * @param minInliersNum 视为有效平面的最小内点数
 * @param MaxIterations 最大迭代次数
 * @param logger ros2 logger
 * @return std::vector<PlaneData> 包含所有找到的平面的信息（系数和点云）
 */
std::vector<PlaneData> segmentPlanesWithPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
    double distanceThreshold,
    int MaxIterations,
    int minInliersNum,
    int maxPlaneNum,
    const rclcpp::Logger& logger=rclcpp::get_logger("segmentPlanesWithPoints"));

// Function to remove hidden points using Depth Buffering
// Input:
//   cloud_camera_frame: Pointer to the input point cloud (MUST BE in camera coordinates)
//   img_width: Width of the virtual image plane (pixels)
//   img_height: Height of the virtual image plane (pixels)
//   fx, fy, cx, cy: Camera intrinsic parameters
//   near_plane: Near clipping plane distance
// Output:
//   A new point cloud containing only the visible points (in camera frame)
pcl::PointCloud<pcl::PointXYZ>::Ptr removeHiddenPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera_frame,
    int img_width, int img_height,
    float fx, float fy, float cx, float cy,
    float near_plane = 0.01f);


/**
 * @brief Calculates the distance from the origin (0, 0, 0) to the plane defined by the coefficients.
 *
 * The plane equation is ax + by + cz + d = 0.
 * The distance from a point (x0, y0, z0) to the plane is |ax0 + by0 + cz0 + d| / sqrt(a^2 + b^2 + c^2).
 * For the origin (0, 0, 0), the distance is |a*0 + b*0 + c*0 + d| / sqrt(a^2 + b^2 + c^2) = |d| / sqrt(a^2 + b^2 + c^2).
 *
 * @param plane_data The PlaneData struct containing the plane coefficients.
 * @return The distance from the origin to the plane. Returns NaN if coefficients are invalid
 *         (not 4 values) or if (a, b, c) vector is zero (not a valid plane normal).
 */
float PlaneDistanceToOrigin(const PlaneData& plane_data);

/**
 * @brief Calculates the distance from the origin (0, 0, 0) to the intersection point
 *        of the plane with a ray starting at the origin and parallel to the positive Z-axis.
 *
 * The plane equation is ax + by + cz + d = 0.
 * The ray is parameterized as (0, 0, t) for t >= 0.
 * Substituting into the plane equation gives c*t + d = 0.
 * If c != 0, t = -d/c. The intersection exists along the positive Z-axis ray if t >= 0.
 * If c == 0, the plane is parallel to the Z-axis. If d == 0, the plane contains the Z-axis ray.
 * If c == 0 and d != 0, the plane is parallel to the Z-axis and does not contain the ray.
 * In cases where c == 0, there is no unique intersection point along the ray.
 *
 * @param plane_data The PlaneData struct containing the plane coefficients.
 * @return The distance 't' if the intersection point is (0, 0, t) with t >= 0.
 *         Returns NaN if there is no such intersection (plane parallel to Z-axis,
 *         or intersection is on the negative Z-axis). Returns NaN for invalid coefficients.
 */
float IntersectionDistanceAlongZ(const PlaneData& plane_data);


/**
 * @brief Determines the plane defined by three given points.
 *        Input points are Eigen::Vector3d (Eigen::Matrix<double, 3, 1>).
 *
 * The plane equation is ax + by + cz + d = 0, where (a, b, c) is the normalized normal vector.
 *
 * @param p1 The first point (Eigen::Vector3d).
 * @param p2 The second point (Eigen::Vector3d).
 * @param p3 The third point (Eigen::Vector3d).
 * @param epsilon Tolerance for checking if points are collinear (normal vector is close to zero).
 * @return A std::vector<double> containing the coefficients {a, b, c, d} if successful (size 4).
 *         Returns an empty std::vector<double> if the three points are collinear.
 */
std::vector<double> determinePlaneFromThreePoints(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double epsilon = 1e-9);

/**
 * @brief Calculates the distance from a point to a plane defined by a vector of coefficients.
 *
 * The plane is defined by coefficients (a, b, c, d) in the vector for the equation ax + by + cz + d = 0.
 * It assumes (a, b, c) is a normalized normal vector, so sqrt(a^2 + b^2 + c^2) ≈ 1.
 * The distance from point (x0, y0, z0) is |a*x0 + b*y0 + c*z0 + d|.
 *
 * @param point The 3D point (x0, y0, z0).
 * @param plane_coefficients A vector of doubles containing the plane coefficients {a, b, c, d}.
 *                           Assumed to have size 4 and (a, b, c) normalized.
 * @return The distance from the point to the plane. Returns NaN if coefficients vector is invalid.
 */
double PointToPlaneDistance(
    const pcl::PointXYZ& point,
    const std::vector<double>& plane_coefficients);

double PointToPlaneDistance(
    const pcl::PointXYZ& point,
    const std::vector<float>& plane_coefficients);

double PointsToPlaneDistance(
    const pcl::PointCloud<pcl::PointXYZ>& point,
    const std::vector<float>& plane_coefficients);
double PointsToPlaneDistance(
    const pcl::PointCloud<pcl::PointXYZ>& point,
    const std::vector<double>& plane_coefficients);

geometry_msgs::msg::TransformStamped ReverseTransforme(
    const geometry_msgs::msg::TransformStamped& transform_A_to_child);

double ReprojectionError(
    cv::Mat & rvec,
    cv::Mat & tvec, 
    const std::vector<cv::Point3d> & objectPoints, 
    const std::vector<cv::Point2d> & imagePoints, 
    const cv::Mat & cameraMatrix, 
    const cv::Mat & distCoeffs=cv::Mat(0,0,0,0,0));

} // end namespace Engineering_robot_RM2025_Pnx

#endif