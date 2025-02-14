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

#include <rclcpp/rclcpp.hpp>
// #include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::placeholders;


class Mid360Driver : public rclcpp::Node {
public:
    Mid360Driver(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
    : Node("mid360_driver", options) {
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/mid360/point_cloud", 10);
        this->declare_parameter<std::string>("mid360_config_Location","/root/Engineering_robot_RM2025_Pnx/src/senserdrivers/config/mid360_config.json");
        configpath=this->get_parameter("mid360_config_Location").as_string();
        RCLCPP_INFO(this->get_logger(),"mid360_configpath:%s",configpath.c_str());

        if (!LivoxLidarSdkInit(configpath.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed.");
            LivoxLidarSdkUninit();
            rclcpp::shutdown();
        }
        else RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");
    }
    ~Mid360Driver() {
        LivoxLidarSdkUninit();
        RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkUninit successfully.");
    }
    void PublishPointCloud(const LivoxLidarEthernetPacket* data);
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std::string configpath;
};

void PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data);
void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);
void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
