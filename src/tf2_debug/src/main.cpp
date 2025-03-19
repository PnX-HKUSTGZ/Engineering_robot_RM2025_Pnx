#include "interfaces/srv/imagerequest.hpp"
#include "interfaces/msg/redeem_box_position.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <chrono>

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class tf2_Debug : public rclcpp::Node{
    public:
    tf2_Debug():Node("tf2_Debug"){
        config=YAML::LoadFile("/home/pnx/code/Engineering_robot_RM2025_Pnx/src/config.yaml");

        camera_to_map.header.frame_id="map";
        camera_to_map.child_frame_id="sensor/camera";
        camera_to_map.header.stamp=this->now();
        camera_to_map.transform.rotation.w=config["object_pos"]["camera"]["rotate"]["w"].as<double>();
        camera_to_map.transform.rotation.x=config["object_pos"]["camera"]["rotate"]["x"].as<double>();
        camera_to_map.transform.rotation.y=config["object_pos"]["camera"]["rotate"]["y"].as<double>();
        camera_to_map.transform.rotation.z=config["object_pos"]["camera"]["rotate"]["z"].as<double>();
        camera_to_map.transform.translation.x=config["object_pos"]["camera"]["translation"]["x"].as<double>();
        camera_to_map.transform.translation.y=config["object_pos"]["camera"]["translation"]["y"].as<double>();
        camera_to_map.transform.translation.z=config["object_pos"]["camera"]["translation"]["z"].as<double>();

        mid360_to_map.header.stamp =this->now();
        mid360_to_map.header.frame_id = "map";
        mid360_to_map.child_frame_id = "sensor/mid360";
        mid360_to_map.transform.translation.x = config["object_pos"]["mid360"]["translation"]["x"].as<double>();
        mid360_to_map.transform.translation.y = config["object_pos"]["mid360"]["translation"]["y"].as<double>();
        mid360_to_map.transform.translation.z = config["object_pos"]["mid360"]["translation"]["z"].as<double>();
        mid360_to_map.transform.rotation.x = config["object_pos"]["mid360"]["rotate"]["x"].as<double>();
        mid360_to_map.transform.rotation.y = config["object_pos"]["mid360"]["rotate"]["y"].as<double>();
        mid360_to_map.transform.rotation.z = config["object_pos"]["mid360"]["rotate"]["z"].as<double>();
        mid360_to_map.transform.rotation.w = config["object_pos"]["mid360"]["rotate"]["w"].as<double>();

        staticpospub=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        staticpospub->sendTransform(camera_to_map);
        staticpospub->sendTransform(mid360_to_map);

        buf=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        lis=std::make_shared<tf2_ros::TransformListener>(*buf,this);

        tme=this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&tf2_Debug::callback,this));

    }

    void callback(){

        geometry_msgs::msg::TransformStamped transform;
        try{
            transform=buf->lookupTransform("sensor/camera","sensor/mid360",this->now());
        }
        catch(tf2::TransformException & ex){
            RCLCPP_WARN(this->get_logger(),"[ImageCloudPointCallBack]: %s",ex.what());
            return;
        }
        std::stringstream sss;
        sss<<"tvec "
        <<transform.transform.translation.x<<" "
        <<transform.transform.translation.y<<" "
        <<transform.transform.translation.z<<" "<<"\n"
        <<"rvec "
        <<transform.transform.rotation.x<<" "
        <<transform.transform.rotation.y<<" "
        <<transform.transform.rotation.z<<" "
        <<transform.transform.rotation.w<<" ";
        RCLCPP_INFO(this->get_logger(),"%s",sss.str().c_str());
    }

    YAML::Node config;
    geometry_msgs::msg::TransformStamped camera_to_map;
    geometry_msgs::msg::TransformStamped mid360_to_map;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticpospub;

    std::shared_ptr<tf2_ros::TransformListener> lis;
    std::shared_ptr<tf2_ros::Buffer> buf;

    std::shared_ptr<rclcpp::TimerBase> tme;
};

int main (int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<tf2_Debug>();

    rclcpp::spin(node);
    rclcpp::shutdown();
}