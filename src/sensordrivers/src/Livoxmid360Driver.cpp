#include "Livoxmid360Driver.hpp"

Engineering_robot_RM2025_Pnx::Mid360Driver::Mid360Driver(const rclcpp::NodeOptions & options) : 
  rclcpp::Node("LivoxMid360Driver", options) {

  // param init
  std::string Location = this->declare_parameter<std::string>("Location", "");
  std::string yaml_path = Location + "/src/config.yaml";
  std::string mid360_config_path = this->declare_parameter<std::string>("mid360_config_path", "");

  if(mid360_config_path.empty()||Location==""){
    RCLCPP_ERROR(this->get_logger(), "mid360_config_path is empty or Location is empty.");
    return;
  }

  // yaml init
  try{
    config = YAML::LoadFile(yaml_path);
  }
  catch(YAML::BadFile& e){
    RCLCPP_ERROR(this->get_logger(), "YAML file not found. %s PATH: %s", e.what(),yaml_path.c_str());
    return;
  }

  // mid360 init
  bool initok=false;
  while(!initok){
    try{
      LivoxLidarSdkInit(mid360_config_path.c_str());
      initok=1;
    }
    catch(std::exception& e){
      initok=0;
      RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed. %s", e.what());
    }
  }
  
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
  SetLivoxLidarImuDataCallback(ImuDataCallback, this);
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
  RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");

  //ros2 publisher broadcaster init
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/mid360/point_cloud", 10);
  tf_static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // tf2 broadcaster init
  geometry_msgs::msg::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = this->now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = frame_id;
  try{
    static_transformStamped.transform.translation.x = config["object_pos"]["mid360"]["translation"]["x"].as<double>();
    static_transformStamped.transform.translation.y = config["object_pos"]["mid360"]["translation"]["y"].as<double>();
    static_transformStamped.transform.translation.z = config["object_pos"]["mid360"]["translation"]["z"].as<double>();
    static_transformStamped.transform.rotation.x = config["object_pos"]["mid360"]["rotate"]["x"].as<double>();
    static_transformStamped.transform.rotation.y = config["object_pos"]["mid360"]["rotate"]["y"].as<double>();
    static_transformStamped.transform.rotation.z = config["object_pos"]["mid360"]["rotate"]["z"].as<double>();
    static_transformStamped.transform.rotation.w = config["object_pos"]["mid360"]["rotate"]["w"].as<double>();
  }
  catch(YAML::BadFile& e){
    RCLCPP_ERROR(this->get_logger(), "tf2 config not found. %s", e.what());
    return;
  }
  tf_static_transform_broadcaster->sendTransform(static_transformStamped);
  RCLCPP_INFO(this->get_logger(), "tf2 broadcaster init successfully.");

  try{
    if(config["debug"]["mid360"].IsDefined()&&config["debug"]["mid360"].as<bool>()){
      DEBUGInit();
    }
  }
  catch(YAML::BadFile& e){
    RCLCPP_ERROR(this->get_logger(), "debug config not found. %s", e.what());
  }

  RCLCPP_INFO(this->get_logger(), "Mid360Driver init successfully.");
}

Engineering_robot_RM2025_Pnx::Mid360Driver::~Mid360Driver() {
  LivoxLidarSdkUninit();
  RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkUninit successfully.");
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  #ifdef cloudelog
  RCLCPP_INFO(node->get_logger(),"PointCloudCallback called.");
  #endif
  if(data == nullptr) {
    return;
  }
  if(data->data_type == kLivoxLidarSphericalCoordinateData) {
    RCLCPP_ERROR(node->get_logger(),"data_type is kLivoxLidarSphericalCoordinateData not supported.");
    return;
  }

  LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for(int i = 0; i < data->dot_num; i++) {
    if(p_point_data[i].tag != 0) continue;
    cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
  }
  pcl::toROSMsg(cloud, cloud_msg);
  
  cloud_msg.header.frame_id = node->frame_id;
  cloud_msg.header.stamp = node->now();
  cloud_msg.height = 1;
  cloud_msg.is_dense = true;
  cloud_msg.width = cloud.size();
  node->point_cloud_pub_->publish(cloud_msg);
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (data == nullptr) {
    return;
  } 
  #ifdef cloudelog
  RCLCPP_INFO(node->get_logger(),"Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  #endif
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (info == nullptr) {
      RCLCPP_ERROR(node->get_logger(),"lidar info change callback failed, the info is nullptr.\n");
      return;
  }
  else RCLCPP_INFO(node->get_logger(),"LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);

  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle; 
  std::stringstream ss; 
  ss << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: ";
  RCLCPP_INFO(node->get_logger(),"%s", ss.str().c_str());
  RCLCPP_INFO(node->get_logger(),"%s", info);
  return;
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (response == nullptr) {
      return;
  }
  RCLCPP_INFO(node->get_logger(),"WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
        status, handle, response->ret_code, response->error_key);
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    Engineering_robot_RM2025_Pnx::Mid360Driver);