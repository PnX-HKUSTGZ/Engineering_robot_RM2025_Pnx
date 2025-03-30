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
#include <queue>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#define cloudelog

namespace Engineering_robot_RM2025_Pnx{

    using namespace std::chrono_literals;
    using namespace std::placeholders;

    class Mid360Driver : public rclcpp::Node {
    public:
        Mid360Driver(const rclcpp::NodeOptions & options=rclcpp::NodeOptions());
        ~Mid360Driver();

    private:
        YAML::Node config;
        std::string frame_id="sensor/mid360";
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_transform_broadcaster;
        
    private: // Callback function
        static void PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
        static void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
        static void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
        static void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
        static void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data);
        static void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
        
    //DEBUG PART
    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_all;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::queue<std::pair<int,rclcpp::Time>> CloudTimeStamp;
        rclcpp::Duration buffertime=rclcpp::Duration(0,0);
        void DEBUGInit();
        void addPoint(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void publishCloud(builtin_interfaces::msg::Time time_);
        void UpdateCloud();
    };

}
