#include "SenserDrivers/Livoxmid360Driver.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

std::shared_ptr<Mid360Driver> node;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  RCLCPP_INFO(node->get_logger(),"PointCloudCallback called.");
  if (data == nullptr) {
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("Mid360Driver:PointCloudCallback"),"point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
    node->PublishPointCloud(data);
}

void Mid360Driver::PublishPointCloud(const LivoxLidarEthernetPacket* data) {
    if(data->data_type == kLivoxLidarSphericalCoordinateData) {
        RCLCPP_ERROR(this->get_logger(),"data_type is kLivoxLidarSphericalCoordinateData not supported.");
        return;
    }
    // node->cloud_buffer_->addPoint(data);
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    int cloud_siz=0;
    for(size_t i=0;i<data->dot_num;i++) cloud_siz += (p_point_data[i].tag == 0);
    cloud.height=1;
    cloud.width=cloud_siz;
    cloud.is_dense=true;
    cloud.points.resize(cloud.height*cloud.width);

    for(size_t i=0,j=0;i<data->dot_num;i++){
        if(p_point_data[i].tag == 0){
            cloud.points[j].x=p_point_data[i].x/1000.0;
            cloud.points[j].y=p_point_data[i].y/1000.0;
            cloud.points[j].z=p_point_data[i].z/1000.0;
            RCLCPP_INFO(this->get_logger(),"point: %f, %f, %f, %f",cloud.points[j].x,cloud.points[j].y,cloud.points[j].z);
            j++;
        }
    }
    pcl::toROSMsg(cloud,cloud_msg);
    cloud_msg.header.frame_id="/sensor/mid360";
    cloud_msg.header.stamp=node->get_clock()->now();
    point_cloud_pub_->publish(cloud_msg);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) {
        RCLCPP_ERROR(node->get_logger(),"lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    else RCLCPP_INFO(node->get_logger(),"LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data){
    if (response == nullptr) {
        return;
    }
    RCLCPP_ERROR(node->get_logger(),"WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
          status, handle, response->ret_code, response->error_key);
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    RCLCPP_ERROR(node->get_logger(),"Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  RCLCPP_INFO(node->get_logger(),"Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  RCLCPP_INFO(node->get_logger(),"Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

int main (int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    node = std::make_shared<Mid360Driver>();
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
    RCLCPP_INFO(node->get_logger(), "SetCallBack successfully.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// test
// int main (int argc, const char *argv[]) {
//   rclcpp::init(argc, argv);
//   auto pnode = std::make_shared<rclcpp::Node>("test");
//   auto pub_=pnode->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/mid360/pointcloud",10);
//   auto static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(pnode);
//   geometry_msgs::msg::TransformStamped t;
//   t.header.stamp =pnode->now();
//   t.header.frame_id = "map";
//   t.child_frame_id = "sensor/mid360";
//   t.transform.translation.x = 0.0;
//   t.transform.translation.y = 0.0;
//   t.transform.translation.z = 0.0;
//   t.transform.rotation.x = 0.0;
//   t.transform.rotation.y = 0.0;
//   t.transform.rotation.z = 0.0;
//   t.transform.rotation.w = 1.0;
//   static_tf_broadcaster_->sendTransform(t);
//   RCLCPP_INFO(pnode->get_logger(), "tf broadcaster successfully.");
//   pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
//   cloud_pcl.width=2;
//   cloud_pcl.height=1;
//   cloud_pcl.points.resize(cloud_pcl.width*cloud_pcl.height);
//   cloud_pcl.points[0].x=1;
//   cloud_pcl.points[0].y=1;
//   cloud_pcl.points[0].z=0;
//   cloud_pcl.points[1].x=-1;
//   cloud_pcl.points[1].y=-1;
//   cloud_pcl.points[1].z=0;
//   sensor_msgs::msg::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud_pcl,cloud_msg);
//   cloud_msg.header.frame_id="/sensor/mid360";
//   cloud_msg.header.stamp=pnode->get_clock()->now();
//   auto a=pnode->create_wall_timer(std::chrono::milliseconds(100),[&](){
//     pub_->publish(cloud_msg);
//     RCLCPP_INFO(pnode->get_logger(),"publish point cloud.");
//   });
//   a->call();
//   rclcpp::spin(pnode);
//   rclcpp::shutdown();
//   return 0;
// }