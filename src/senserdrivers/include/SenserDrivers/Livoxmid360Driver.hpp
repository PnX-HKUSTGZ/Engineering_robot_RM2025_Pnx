#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

// #define cloudelog

using namespace std::placeholders;

using namespace std::placeholders;


class Mid360Driver : public rclcpp::Node {
public:
    Mid360Driver(const rclcpp::NodeOptions & options=rclcpp::NodeOptions());
    ~Mid360Driver() {
        LivoxLidarSdkUninit();
        RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkUninit successfully.");
    }
    void PublishPointCloud(const LivoxLidarEthernetPacket* data);
    void PublishIMU(const LivoxLidarEthernetPacket* data);
private:
    void synchronous_pose(sensor_msgs::msg::Imu::SharedPtr msg);
    void update_pose_rotate(double dt,sensor_msgs::msg::Imu::SharedPtr msg);
    void update_pose_translate(double dt,sensor_msgs::msg::Imu::SharedPtr msg);
    void pub_pose(rclcpp::Time time=rclcpp::Clock().now());
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_transform_broadcaster;
    rclcpp::TimerBase::SharedPtr cloud_buffer_timer_; 
    YAML::Node config;

    // mid360 special config
    std::string configpath;


    public:
    void addPoint(const LivoxLidarEthernetPacket* data);

    void publishCloud(builtin_interfaces::msg::Time time_=rclcpp::Clock().now());

    void reset();

    bool isoutoftime();

private:
    rclcpp::Time start_time;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string frame_id;
    int buffertime;
};

void PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data);
void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);
void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
