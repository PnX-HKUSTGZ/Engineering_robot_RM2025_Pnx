#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
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
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/utils.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <logging.hpp>

using namespace std::chrono_literals;

const double eps=1e-9;

class DepthFusion : public rclcpp::Node {
public:

DepthFusion() : Node("depth_fusion"){
    cloud_sub_.subscribe(this,"/sensor/mid360/point_cloud");
    image_sub_.subscribe(this,"sensor/image");
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"sub_ create!.");

    // 初始化tf2
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"tf2_ create!.");

    // 初始化同步器
    sync_.reset(new Sync(SyncPolicy(10), cloud_sub_, image_sub_));
    sync_->registerCallback(&DepthFusion::callback, this);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"sync_ create!.");

    // 初始化深度图的发布者
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sensor/depimage",10);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"depth_pub_ create!.");

    // 初始化点云的发布者
    point_colored_cloud_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/colored_pointcloud",10);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"point_colored_cloud_pub_ create!.");

    // 初始化tf2的静态广播器
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp =this->now();
    t.header.frame_id = "/sensor/camera";
    t.child_frame_id = "/sensor/colored_pointcloud";
    t.transform.translation.x =0;
    t.transform.translation.y =0;
    t.transform.translation.z =0;
    t.transform.rotation.x =0;
    t.transform.rotation.y =0;
    t.transform.rotation.z =0;
    t.transform.rotation.w =1;
    static_tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO_EXPRESSION(this->get_logger(),1,"tf broadcaster successfully.");

    //定义参数
    this->declare_parameter<std::string>("Location","/home/lqx/code/Engineering_robot_RM2025_Pnx");
    YAML::Node config = YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");

    camera_matrix_ = config["camera"]["camera_matrix"].as<std::vector<double>>();
    dist_coeffs_ = config["camera"]["dist_coeffs"].as<std::vector<double>>();
    Eigen::Matrix3d lin_camera_matrix_=Eigen::Matrix3d::Zero();
    lin_camera_matrix_<<camera_matrix_[0],camera_matrix_[1],camera_matrix_[4],
                      camera_matrix_[2],camera_matrix_[3],camera_matrix_[5],
                      camera_matrix_[6],camera_matrix_[7],camera_matrix_[8];
    camera_matrix_eigen_=Eigen::Matrix4d::Zero();
    camera_matrix_eigen_.block<3,3>(0,0)=lin_camera_matrix_;
    camera_matrix_Mat=cv::Mat(3,3,CV_64FC1,camera_matrix_.data());
    height = config["camera"]["height"].as<int>();
    width = config["camera"]["width"].as<int>();

}

private:

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
              const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
        // if(this->colored_point.size()>100000){
        //     return;
        // }
        try{
            tf2::TimePoint image_time_point = tf2::TimePoint(std::chrono::seconds(image_msg->header.stamp.sec)+std::chrono::nanoseconds(image_msg->header.stamp.nanosec));
            tf2::TimePoint cloud_time_point = tf2::TimePoint(std::chrono::seconds(cloud_msg->header.stamp.sec)+std::chrono::nanoseconds(cloud_msg->header.stamp.nanosec));
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("sensor/camera", image_time_point, "sensor/mid360", cloud_time_point, "map");
            // 定义四元数 [w, x, y, z]
            Eigen::Quaternion rotate(transform.transform.rotation.w,transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z);
            Eigen::Matrix3d rotation_matrix = rotate.toRotationMatrix();
            Eigen::Vector3d translation(transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z);
            Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
            transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
            transform_matrix.block<3, 1>(0, 3) = translation;
            std::stringstream ss;ss << transform_matrix;
            RCLCPP_INFO(get_logger(),"1");
            RCLCPP_INFO(this->get_logger(),"transform_matrix: %s",ss.str().c_str());

            pcl::PointCloud<pcl::PointXYZ> pcl_cloud,pcl_cloud_transformed;

            // convert ros msg to pcl cloud.
            // fromMsgPointCloud2toPointCloud2(*cloud_msg,ros_cloud);
            pcl::fromROSMsg<pcl::PointXYZ>(*cloud_msg,pcl_cloud);
            
            RCLCPP_INFO(get_logger(),"fromROSMsg");
            // transform
            RCLCPP_INFO(this->get_logger(),"size: %d %d %d",pcl_cloud.size(),pcl_cloud.width,pcl_cloud.height);
            if(pcl_cloud.size()==0) return;
            for (int i = 0,len=pcl_cloud.points.size(); i < len; ++i) {
                RCLCPP_INFO(this->get_logger(),"point: %f, %f, %f, %f",pcl_cloud.points[i].x,pcl_cloud.points[i].y,pcl_cloud.points[i].z);
            }
            pcl::transformPointCloud(pcl_cloud,pcl_cloud_transformed,transform_matrix);
            RCLCPP_INFO(get_logger(),"transformPointCloud");

            //转换得到opencv::Mat 并处理畸变
            cv::Mat cv_image=cv_bridge::toCvCopy(image_msg,"bgr8")->image;
            cv::Mat cv_depth(cv_image.size(),CV_32FC1),cv_image_undistort;
            cv::undistort(cv_image,cv_image_undistort,camera_matrix_Mat,dist_coeffs_);
            // cv::imshow("???",cv_image_undistort);
            // cv::imshow("??",cv_image);
            // cv::waitKey(30);
            // 遍历点云，将每个点投影到图像上
            RCLCPP_INFO(get_logger(),"undistort");
            for (int i = 0,len=pcl_cloud_transformed.points.size(); i < len; ++i) {
                // 将点云坐标转换为像素坐标
                const pcl::PointXYZ &point = pcl_cloud_transformed.points[i];
                if(point.z<=0){
                    continue;
                }
                Eigen::Vector4d point_homogeneous(point.x, point.y, point.z, 1.0);
                Eigen::Vector4d point_camera = camera_matrix_eigen_ * point_homogeneous;
                if(!(point_camera(0,0)>=0 && point_camera(1,0)>=0 && point_camera(0,0)<height && point_camera(1,0)<width)) continue;
                if(point_camera(2,0)<cv_depth.at<float>(point_camera(1,0),point_camera(0,0))||std::abs(cv_depth.at<float>(point_camera(1,0),point_camera(0,0)))<=eps){
                    cv::Point2d targ=cv::Point2d(point_camera(1,0),point_camera(0,0));
                    this->colored_point.push_back(pcl::PointXYZRGB(point.x,point.y,point.z,
                        cv_image_undistort.at<cv::Vec3b>(targ)[2],cv_image_undistort.at<cv::Vec3b>(targ)[1],cv_image_undistort.at<cv::Vec3b>(targ)[0]));
                    std::stringstream ss;
                    ss<<targ<<" "<<cv_image_undistort.at<cv::Vec3b>(targ)[2]<<" "<<cv_image_undistort.at<cv::Vec3b>(targ)[1]<<" "<<cv_image_undistort.at<cv::Vec3b>(targ)[0];
                    ss<<std::endl<<point_homogeneous;
                    ss<<std::endl<<"camera_matrix_eigen_ :\n"<<camera_matrix_eigen_;
                    ss<<std::endl<<"point_camera :\n"<<point_camera;
                    RCLCPP_INFO(this->get_logger(),"rgb : %s",ss.str().c_str());
                    cv_depth.at<float>(point_camera(1,0),point_camera(0,0))=point_camera(2,0);
                }
            }
            this->colored_point.height=1;
            this->colored_point.width=colored_point.points.size();
            this->colored_point.is_dense=true;
            RCLCPP_INFO(get_logger(),"setting");
            sensor_msgs::msg::PointCloud2 colored_point_msg;
            pcl::toROSMsg(this->colored_point,colored_point_msg);
            colored_point_msg.header.frame_id="/sensor/colored_pointcloud";
            colored_point_msg.header.stamp=image_msg->header.stamp;
            point_colored_cloud_pub_->publish(colored_point_msg);
            RCLCPP_INFO(get_logger(),"publish");

            cv::Mat cv_depth_normalized;
            double max_depth=0;
            cv::minMaxLoc(cv_depth,NULL,&max_depth,NULL);
            cv::convertScaleAbs(cv_depth,cv_depth_normalized,255.0/max_depth);
            // cv::imshow("depth",cv_depth_normalized);
            // cv::waitKey(30);
            // 发布深度图
            sensor_msgs::msg::Image::SharedPtr depth_msg=cv_bridge::CvImage(image_msg->header,sensor_msgs::image_encodings::TYPE_32FC1,cv_depth_normalized).toImageMsg();
            depth_pub_->publish(*depth_msg);
            RCLCPP_INFO(get_logger(),"CvImage");
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

// 点云的发布者
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_colored_cloud_pub_;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
pcl::PointCloud<pcl::PointXYZRGB> colored_point;

//相机参数
std::vector<double> camera_matrix_;
cv::Mat camera_matrix_Mat;
std::vector<double> dist_coeffs_;
Eigen::Matrix4d camera_matrix_eigen_;
int height;
int width;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthFusion>());
    rclcpp::shutdown();
    return 0;
}