#include "SenserDrivers/Livoxmid360Driver.hpp"

std::shared_ptr<Mid360Driver> node;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
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
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.height = 1; //无序点云
    cloud_msg.width = data->dot_num; // 点云数量
    cloud_msg.is_dense = true; // 点云是否有序
    cloud_msg.is_bigendian = false; // 字节序

    //设置内容
    if(data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        cloud_msg.fields.resize(5);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::INT32;
        cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::INT32;
        cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::INT32;
        cloud_msg.fields[2].count = 1;
        cloud_msg.fields[3].name = "reflectivity";
        cloud_msg.fields[3].offset = 12;
        cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
        cloud_msg.fields[3].count = 1;
        cloud_msg.fields[4].name = "tag";
        cloud_msg.fields[4].offset = 14;
        cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
        cloud_msg.fields[4].count = 1;
        cloud_msg.point_step = 16; // 每个点的大小
    }
    else if(data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        cloud_msg.fields.resize(5);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::INT16;
        cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 2;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::INT16;
        cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 4;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::INT16;
        cloud_msg.fields[2].count = 1;
        cloud_msg.fields[3].name = "reflectivity";
        cloud_msg.fields[3].offset = 6;
        cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
        cloud_msg.fields[3].count = 1;
        cloud_msg.fields[4].name = "tag";
        cloud_msg.fields[4].offset = 7;
        cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
        cloud_msg.fields[4].count = 1;
        cloud_msg.point_step = 8; // 每个点的大小
    }

    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width; // 每行的大小
    cloud_msg.data.resize(cloud_msg.row_step); // 分配内存
    memcpy(cloud_msg.data.data(), p_point_data, cloud_msg.row_step); // 复制数据
    point_cloud_pub_->publish(cloud_msg); // 发布消息
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