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
#include <tf2/utils.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::placeholders;

class CloudPointBuffer {
public:
    CloudPointBuffer(float duration, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_, rclcpp::Node* node):
        duration(duration), frame_id(frame_id), point_cloud_pub_(point_cloud_pub_), node(node){
        start_time=node->now();
    }
    void addPoint(const LivoxLidarEthernetPacket* data) {
        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
        if(node->now()-start_time>rclcpp::Duration::from_seconds(duration)){
            RCLCPP_INFO(node->get_logger(),"addPoint time out, this call will not publish cloud.");
            return;
        }
        else RCLCPP_INFO(node->get_logger(),"addPoint time in, this call will publish cloud.");
        mtx_cloud.lock();
        int cloud_siz=0;
        for(size_t i=0;i<data->dot_num;i++) cloud_siz += (p_point_data[i].tag == 0);
        for(size_t i=0;i<cloud_siz;i++){
            if(p_point_data[i].tag != 0) continue;
            cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
            RCLCPP_INFO(node->get_logger(),"point: %lf, %lf, %lf, %lf",cloud.points.back().x,cloud.points.back().y,cloud.points.back().z);
        }
        cloud.width=cloud.size();
        cloud.height=1;
        cloud.is_dense=true;
        mtx_cloud.unlock();
        RCLCPP_INFO(node->get_logger(),"addPoint successfully.");
    }

    void publishCloud() {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        mtx_cloud.lock();
        pcl::toROSMsg(cloud,cloud_msg);
        mtx_cloud.unlock();
        cloud_msg.header.frame_id=frame_id;
        cloud_msg.header.stamp=node->now();
        point_cloud_pub_->publish(cloud_msg);
        this->reset();
        RCLCPP_INFO(node->get_logger(),"publishCloud successfully.");
    }

    void reset() {
        start_time=node->now();
        cloud.clear();
    }

    bool isoutoftime() {
        return node->now()-start_time>rclcpp::Duration::from_seconds(duration);
    }

private:
    rclcpp::Time start_time;
    float duration; // unit : second
    pcl::PointCloud<pcl::PointXYZ> cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Node* node;
    const std::string frame_id;
    // mutex for cloud
    std::mutex mtx_cloud;
};

class Mid360Driver : public rclcpp::Node {
public:
    Mid360Driver(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
    : Node("mid360_driver", options) {
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/mid360/point_cloud", 10);
        this->declare_parameter<std::string>("mid360_config_Location","/home/lqx/code/Engineering_robot_RM2025_Pnx/src/senserdrivers/config/mid360_config.json");
        configpath=this->get_parameter("mid360_config_Location").as_string();
        RCLCPP_INFO(this->get_logger(),"mid360_configpath:%s",configpath.c_str());

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp =this->now();
        t.header.frame_id = "map";
        t.child_frame_id = "sensor/mid360";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "tf broadcaster successfully.");

        if (!LivoxLidarSdkInit(configpath.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed.");
            LivoxLidarSdkUninit();
            rclcpp::shutdown();
        }
        else RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");

        cloud_buffer_=std::make_shared<CloudPointBuffer>(1,"sensor/mid360",point_cloud_pub_,this);

        cloud_buffer_timer_=this->create_wall_timer(std::chrono::milliseconds(100),[&](){
            if(cloud_buffer_->isoutoftime())  cloud_buffer_->publishCloud();
        });

    }
    ~Mid360Driver() {
        LivoxLidarSdkUninit();
        RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkUninit successfully.");
    }
    void PublishPointCloud(const LivoxLidarEthernetPacket* data);
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::string configpath;
    std::shared_ptr<CloudPointBuffer> cloud_buffer_;
    rclcpp::TimerBase::SharedPtr cloud_buffer_timer_;
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
