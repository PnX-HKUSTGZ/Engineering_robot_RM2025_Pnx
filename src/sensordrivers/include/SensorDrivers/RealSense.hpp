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
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2/utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <librealsense2/rs.hpp>

#include <yaml-cpp/yaml.h>
#include "SensorDrivers/MvCameraControl.h"

namespace Engineering_robot_RM2025_Pnx{

class RealSense: public rclcpp::Node{

public:

    RealSense(rclcpp::NodeOptions=rclcpp::NodeOptions());

private:

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pc_pub_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_pub_;

    std::shared_ptr<std::thread> deal_pipe_thread_;

    // rs2 pip for encapsulating the actual device and sensors
    std::shared_ptr<rs2::pipeline> pipe_;
    // rs2 pointcloud class for calculating pointclouds and texture mappings
    std::shared_ptr<rs2::pointcloud> pc_;
    // rs2 points for pointcloud store
    rs2::points points;

    int depth_wight;
    int depth_hight;
    // for the filter for pointcloud , unit: m
    double depmax,depmin;

    void RS_image_pc_pub_callback();

};

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);

// output order xyzw
std::vector<double> rotationMatrixToQuaternion(const std::vector<double> & matrix);
// output order xyzw
std::vector<double> rotationMatrixToQuaternion(const float matrix[9]);

pcl::PointCloud<pcl::PointXYZ>::Ptr filterDepthRange(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    float depmin,
    float depmax);

}// Engineering_robot_RM2025_Pnx