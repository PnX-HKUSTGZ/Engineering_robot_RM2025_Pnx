#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class DepthFusion : public rclcpp::Node {
public:
    DepthFusion() : Node("depth_fusion"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // 订阅同步话题
        cloud_sub_.subscribe(this, "/lidar/points");
        image_sub_.subscribe(this, "/camera/image_raw");
        
        sync_.reset(new Sync(SyncPolicy(10), cloud_sub_, image_sub_));
        sync_->registerCallback(&DepthFusion::callback, this);

        // 发布深度图
        depth_pub_ = image_transport::create_publisher(this, "/depth/image");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
        try {
            // 获取坐标变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "camera_frame", cloud_msg->header.frame_id, cloud_msg->header.stamp);

            // 转换点云到相机坐标系
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);
            
            Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
            // 填充旋转矩阵和平移向量（根据实际TF数据）
            // ...

            pcl::transformPointCloud(pcl_cloud, pcl_cloud, tf_matrix);

            // 转换为OpenCV图像
            cv::Mat image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            cv::Mat depth_map = cv::Mat::zeros(image.size(), CV_32FC1);

            // 相机内参（需根据实际相机标定填写）
            float fx = 525.0;  // 焦距x
            float fy = 525.0;  // 焦距y
            float cx = 319.5;  // 光心x
            float cy = 239.5;  // 光心y

            // 投影点云到图像平面
            for (const auto& point : pcl_cloud) {
                if (point.z <= 0) continue;

                // 投影计算
                int u = static_cast<int>((fx * point.x / point.z) + cx);
                int v = static_cast<int>((fy * point.y / point.z) + cy);

                // 检查边界
                if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
                    float depth = point.z;  // 使用Z轴作为深度
                    if (depth < depth_map.at<float>(v, u) || depth_map.at<float>(v, u) == 0) {
                        depth_map.at<float>(v, u) = depth;
                    }
                }
            }

            // 转换并发布深度图
            auto depth_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "32FC1", depth_map).toImageMsg();
            depth_pub_.publish(depth_msg);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }

    // 成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    image_transport::Publisher depth_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthFusion>());
    rclcpp::shutdown();
    return 0;
}
