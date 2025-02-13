#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

const double eps=1e-9;

void fromMsgPointCloud2toPointCloud2(const sensor_msgs::msg::PointCloud2& msgcloud, sensor_msgs::PointCloud2& cloud){
    cloud.data=msgcloud.data;
    cloud.height=msgcloud.height;
    cloud.width=msgcloud.width;
    cloud.is_bigendian=msgcloud.is_bigendian;
    cloud.is_dense=msgcloud.is_dense;
    cloud.point_step=msgcloud.point_step;
    cloud.row_step=msgcloud.row_step;
    for (auto i : msgcloud.fields){
        sensor_msgs::PointField lin;
        lin.name=i.name;
        lin.datatype=i.datatype;
        lin.offset=i.offset;
        lin.count=i.count;
        cloud.fields.push_back(lin);
    }
}

class DepthFusion : public rclcpp::Node {
public:

DepthFusion() : Node("depth_fusion"){
    //自动添加命名空间
    ros::NodeHandle nh("sensor");
    cloud_sub_.subscribe(nh,"pointcloud",10);
    image_sub_.subscribe(nh,"image",10);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),rclcpp::ok,"sub_ create!.");
    RCLCPP_ERROR_EXPRESSION(this->get_logger(),!rclcpp::ok,"sub_ create fail!.");

    // 初始化tf2
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),rclcpp::ok,"tf2_ create!.");
    RCLCPP_ERROR_EXPRESSION(this->get_logger(),!rclcpp::ok,"tf2_ create fail!.");

    // 初始化同步器
    sync_.reset(new Sync(SyncPolicy(10), cloud_sub_, image_sub_));
    sync_->registerCallback(&DepthFusion::callback, this);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),rclcpp::ok,"sync_ create!.");
    RCLCPP_ERROR_EXPRESSION(this->get_logger(),!rclcpp::ok,"sync_ create fail!.");

    // 初始化深度图的发布者
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sensor/depimage",10);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),rclcpp::ok,"depth_pub_ create!.");
    RCLCPP_ERROR_EXPRESSION(this->get_logger(),!rclcpp::ok,"depth_pub_ create fail!.");

    //定义参数
    this->declare_parameter<std::string>("Location","/home/lqx/code/Engineering_robot_RM2025_Pnx/config.yaml");
    YAML::Node config = YAML::LoadFile(this->get_parameter("Location").as_string());

    camera_matrix_ = config["camera_matrix"].as<std::vector<double>>();
    dist_coeffs_ = config["dist_coeffs"].as<std::vector<double>>();
    Eigen::Matrix3d lin_dist_coeffs_=Eigen::Matrix3d::Zero();
    lin_dist_coeffs_<<dist_coeffs_[0],dist_coeffs_[1],dist_coeffs_[4],
                      dist_coeffs_[2],dist_coeffs_[3],dist_coeffs_[5],
                      dist_coeffs_[6],dist_coeffs_[7],dist_coeffs_[8];
    camera_matrix_eigen_=Eigen::Matrix4d::Zero();
    camera_matrix_eigen_.block<3,3>(0,0)=lin_dist_coeffs_;
    camera_matrix_Mat=cv::Mat(3,3,CV_64FC1,camera_matrix_.data());
    height = config["height"].as<int>();
    width = config["width"].as<int>();

}

private:

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
              const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
        try{
            tf2::TimePoint image_time_point = tf2::TimePoint(std::chrono::seconds(image_msg->header.stamp.sec)+std::chrono::nanoseconds(image_msg->header.stamp.nanosec));
            tf2::TimePoint cloud_time_point = tf2::TimePoint(std::chrono::seconds(cloud_msg->header.stamp.sec)+std::chrono::nanoseconds(cloud_msg->header.stamp.nanosec));
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("/sensor/camera", image_time_point, "/sensor/pointcloud", cloud_time_point, "odom");
            // 定义四元数 [w, x, y, z]
            Eigen::Quaternion rotate(transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z);
            Eigen::Matrix3d rotation_matrix = rotate.toRotationMatrix();
            Eigen::Vector3d translation(transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z);
            Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
            transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
            transform_matrix.block<3, 1>(0, 3) = translation;
            RCLCPP_INFO(this->get_logger(),"transform_matrix: %f",transform_matrix);

            pcl::PointCloud<pcl::PointXYZ> pcl_cloud,pcl_cloud_transformed;
            sensor_msgs::PointCloud2 ros_cloud;

            // convert ros msg to pcl cloud.
            fromMsgPointCloud2toPointCloud2(*cloud_msg,ros_cloud);
            pcl::fromROSMsg<pcl::PointXYZ>(ros_cloud,pcl_cloud);
            
            // transform
            pcl::transformPointCloud(pcl_cloud,pcl_cloud_transformed,transform_matrix);

            //转换得到opencv::Mat 并处理畸变
            cv::Mat cv_image=cv_bridge::toCvCopy(image_msg,"bgr8")->image;
            cv::Mat cv_depth(cv_image.size(),CV_32FC1),cv_image_undistort;
            cv::undistort(cv_image,cv_image_undistort,camera_matrix_Mat,dist_coeffs_);

            // 遍历点云，将每个点投影到图像上
            for (const auto& point : pcl_cloud_transformed) {
                // 将点云坐标转换为像素坐标
                if(point.z<=0){
                    continue;
                }
                Eigen::Vector4d point_homogeneous(point.x, point.y, point.z, 1.0);
                Eigen::Vector4d point_camera = camera_matrix_eigen_ * point_homogeneous;
                if(!(point_camera(0,0)>=0 && point_camera(1,0)>=0 && point_camera(0,0)<width && point_camera(1,0)<height)) continue;
                if(point_camera(2,0)<cv_depth.at<float>(point_camera(1,0),point_camera(0,0))||std::abs(cv_depth.at<float>(point_camera(1,0),point_camera(0,0)))<=eps){
                    cv_depth.at<float>(point_camera(1,0),point_camera(0,0))=point_camera(2,0);
                }
            }

            cv::Mat cv_depth_normalized;
            double max_depth=0;
            cv::minMaxLoc(cv_depth,NULL,&max_depth,NULL);
            cv::convertScaleAbs(cv_depth,cv_depth_normalized,255.0/max_depth);
            cv::imshow("depth",cv_depth_normalized);
            cv::waitKey(30);
            // 发布深度图
            sensor_msgs::msg::Image::SharedPtr depth_msg=cv_bridge::CvImage(image_msg->header,sensor_msgs::image_encodings::TYPE_32FC1,cv_depth_normalized).toImageMsg();
            depth_pub_->publish(*depth_msg);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }


    }

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;

// 定义同步策略ApproximateTime 是粗略相同步
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;
std::shared_ptr<Sync> sync_;

// 深度图的发布者
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

//相机参数
std::vector<double> camera_matrix_;
cv::Mat camera_matrix_Mat;
std::vector<double> dist_coeffs_;
Eigen::Matrix4d camera_matrix_eigen_;
int height;
int width;
};

int mian(int argc, char **argv){
    rclcpp::init(argc, argv);

}