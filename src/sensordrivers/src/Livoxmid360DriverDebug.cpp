#include "Livoxmid360Driver.hpp"

void Engineering_robot_RM2025_Pnx::Mid360Driver::DEBUGInit(){
    point_cloud_pub_all = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/mid360/point_cloud_all", 10);
    point_cloud_sub_=this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor/mid360/point_cloud", 10,
    std::bind(&Mid360Driver::addPoint, this,_1));
    buffertime=rclcpp::Duration(config["mid_360"]["buffertime"].as<std::vector<int>>()[0],config["mid_360"]["buffertime"].as<std::vector<int>>()[1]) ;
    RCLCPP_INFO(this->get_logger(), "DEBUGInit successfully.");
    return;
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::addPoint(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    UpdateCloud();
    CloudTimeStamp.push(std::make_pair(cloud.size(),msg->header.stamp));
    pcl::PointCloud<pcl::PointXYZ> cloud_now;
    pcl::fromROSMsg(*msg,cloud_now);
    cloud.insert(cloud.end(),cloud_now.begin(),cloud_now.end());
    cloud.width=cloud.size();
    cloud.height=1;
    cloud.is_dense=true;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);
    cloud_msg.header.frame_id=frame_id;
    cloud_msg.header.stamp=this->now();
    point_cloud_pub_all->publish(cloud_msg);
    return;
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::UpdateCloud(){
    while(!CloudTimeStamp.empty()&&(this->now()-CloudTimeStamp.front().second)>buffertime){
      cloud.erase(cloud.begin(),cloud.begin()+CloudTimeStamp.front().first);
      this->CloudTimeStamp.pop();
    }
}