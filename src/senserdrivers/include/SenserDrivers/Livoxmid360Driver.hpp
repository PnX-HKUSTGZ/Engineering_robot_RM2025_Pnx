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

// #define cloudelog

using namespace std::placeholders;

using namespace std::placeholders;

// class CloudPointBuffer {
// public:
//     CloudPointBuffer(float duration, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_, rclcpp::Node* node):
//         duration(duration), frame_id(frame_id), point_cloud_pub_(point_cloud_pub_), node(node){
//         start_time=node->now();
//     }
//     void addPoint(const LivoxLidarEthernetPacket* data) {
//         if(node->now()-start_time>rclcpp::Duration::from_seconds(duration)){
//             RCLCPP_INFO(node->get_logger(),"addPoint time out, this call will not publish cloud.");
//             return;
//         }
//         else RCLCPP_INFO(node->get_logger(),"addPoint time in, this call will publish cloud.");
//         LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
//         // mtx_cloud.lock();
//         int cloud_siz=0;
//         for(size_t i=0;i<data->dot_num;i++){
//             if(p_point_data[i].tag != 0) continue;
//             cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
//             // RCLCPP_INFO(node->get_logger(),"point: %f, %f, %f, %f",cloud.points.back().x,cloud.points.back().y,cloud.points.back().z);
//         }
//         cloud.width=cloud.size();
//         cloud.height=1;
//         cloud.is_dense=true;
//         // mtx_cloud.unlock();
//         RCLCPP_INFO(node->get_logger(),"addPoint successfully.");
//     }

//     void publishCloud(builtin_interfaces::msg::Time time_=rclcpp::Clock().now()) {
//         sensor_msgs::msg::PointCloud2 cloud_msg;
//         // mtx_cloud.lock();
//         pcl::toROSMsg(cloud,cloud_msg);
//         // mtx_cloud.unlock();
//         this->reset();
//         cloud_msg.header.frame_id=frame_id;
//         cloud_msg.header.stamp=time_;
//         point_cloud_pub_->publish(cloud_msg);
//         RCLCPP_INFO(node->get_logger(),"publishCloud successfully.");
//     }

//     void reset() {
//         // mtx_cloud.lock();
//         RCLCPP_INFO(node->get_logger(),"reset cloud buffer.");
//         start_time=node->now();
//         cloud.clear();
//         // mtx_cloud.unlock();
//     }

//     bool isoutoftime() {
//         // mtx_cloud.lock();
//         bool res=node->now()-start_time>rclcpp::Duration::from_seconds(duration);
//         // mtx_cloud.unlock();
//         return res;
//     }

// private:
//     rclcpp::Time start_time;
//     float duration; // unit : second
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
//     rclcpp::Node* node;
//     const std::string frame_id;
//     // mutex for cloud
//     std::mutex mtx_cloud;
// };

class Mid360Driver : public rclcpp::Node {
public:
    Mid360Driver(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
    : Node("mid360_driver", options) {
        // init pose
        pose_rotate=Eigen::Vector4d::Zero();
        pose_rotate(0)=1;
        pose_translate=Eigen::Vector3d::Zero();
        speed_translate=Eigen::Vector3d::Zero();

        this->declare_parameter<double>("g",9.80665);
        g=this->get_parameter("g").as_double();
        
        this->declare_parameter<double>("alpha",0.98);
        alpha=this->get_parameter("alpha").as_double();

        this->declare_parameter<std::string>("mid360_config_Location","/home/lqx/code/Engineering_robot_RM2025_Pnx/src/senserdrivers/config/mid360_config.json");
        configpath=this->get_parameter("mid360_config_Location").as_string();
        RCLCPP_INFO(this->get_logger(),"mid360_configpath:%s",configpath.c_str());

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->pub_pose();
        RCLCPP_INFO(this->get_logger(), "tf broadcaster successfully.");

        if (!LivoxLidarSdkInit(configpath.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed.");
            LivoxLidarSdkUninit();
            rclcpp::shutdown();
        }
        else RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");

        cloud_buffer_timer_=this->create_wall_timer(std::chrono::milliseconds(200),[&](){
            // if(isoutoftime())  publishCloud();
        });

        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/mid360/point_cloud", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor/mid360/imu", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("sensor/mid360/imu", 10, std::bind(&Mid360Driver::synchronous_pose,this,_1));

        image_sub_=this->create_subscription<sensor_msgs::msg::Image>("sensor/image",10,[&](sensor_msgs::msg::Image::SharedPtr msg){
            this->publishCloud(msg->header.stamp);
        });
        duration=0.3;
        frame_id="sensor/mid360";
        start_time=this->now();

    }
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string configpath;
    rclcpp::TimerBase::SharedPtr cloud_buffer_timer_;
    Eigen::Vector4d pose_rotate;
    Eigen::Vector3d pose_translate;
    Eigen::Vector3d speed_translate;
    //重力加速度
    double g=9.80665;
    // 陀螺仪权重
    double alpha = 0.98;  


    public:
    void addPoint(const LivoxLidarEthernetPacket* data) {
        if(this->now()-start_time>rclcpp::Duration::from_seconds(duration)){
            #ifdef cloudelog
            RCLCPP_INFO(this->get_logger(),"addPoint time out, this call will not publish cloud.");
            #endif
            return;
        }
        else {
            #ifdef cloudelog
            RCLCPP_INFO(this->get_logger(),"addPoint time in, this call will publish cloud.");
            #endif
        }
        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
        // mtx_cloud.lock();
        int cloud_siz=0;
        for(size_t i=0;i<data->dot_num;i++){
            if(p_point_data[i].tag != 0) continue;
            cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
            // RCLCPP_INFO(node->get_logger(),"point: %f, %f, %f, %f",cloud.points.back().x,cloud.points.back().y,cloud.points.back().z);
        }
        cloud.width=cloud.size();
        cloud.height=1;
        cloud.is_dense=true;
        // mtx_cloud.unlock();
        #ifdef cloudelog
        RCLCPP_INFO(this->get_logger(),"addPoint successfully.");
        #endif
    }

    void publishCloud(builtin_interfaces::msg::Time time_=rclcpp::Clock().now()) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        // mtx_cloud.lock();
        pcl::toROSMsg(cloud,cloud_msg);
        // mtx_cloud.unlock();
        this->reset();
        cloud_msg.header.frame_id=frame_id;
        cloud_msg.header.stamp=time_;
        point_cloud_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(),"publishCloud successfully.");
    }

    void reset() {
        // mtx_cloud.lock();
        #ifdef cloudelog
        RCLCPP_INFO(this->get_logger(),"reset cloud buffer.");
        #endif
        start_time=this->now();
        cloud.clear();
        // mtx_cloud.unlock();
    }

    bool isoutoftime() {
        // mtx_cloud.lock();
        bool res=this->now()-start_time>rclcpp::Duration::from_seconds(duration);
        // mtx_cloud.unlock();
        return res;
    }

private:
    rclcpp::Time start_time;
    float duration; // unit : second
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string frame_id;
    // mutex for cloud
    std::mutex mtx_cloud;
};

void PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data);
void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);
void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data);
