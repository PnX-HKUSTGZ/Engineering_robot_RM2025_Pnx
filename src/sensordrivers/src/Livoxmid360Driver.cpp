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

  RCLCPP_INFO(this->get_logger(), "Location: %s", Location.c_str());
  RCLCPP_INFO(this->get_logger(), "mid360_config_path: %s", mid360_config_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Load config from YAML file: %s", yaml_path.c_str());

  // yaml init
  try{
    config = YAML::LoadFile(yaml_path);
  }
  catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "error reading config file: %s", e.what());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Config load successfully.");



  //ros2 publisher broadcaster init
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/mid360/point_cloud", 10);
  RCLCPP_INFO(this->get_logger(), "point_cloud_pub_ create successfully.");
  
  tf_static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  RCLCPP_INFO(this->get_logger(), "tf2_static_transform_broadcaster create successfully.");

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
  catch(const std::exception& e){
    RCLCPP_ERROR(this->get_logger(), "tf2 config not found. %s", e.what());
    return;
  }
  tf_static_transform_broadcaster->sendTransform(static_transformStamped);
  RCLCPP_INFO(this->get_logger(), "tf2 broadcaster init successfully.");


  
  // mid360InitThread=std::make_shared<std::thread>([this,mid360_config_path](){
  //   // mid360 init
  //   std::this_thread::sleep_for(1s);
  //   while(!LivoxLidarSdkInit(mid360_config_path.c_str())){
  //     RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit failed, retry after 0.2s.");
  //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
  //     LivoxLidarSdkUninit();
  //   }
  //   RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");
  //   SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
  //   SetLivoxLidarImuDataCallback(ImuDataCallback, this);
  //   SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, this);
  //   SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
  //   RCLCPP_INFO(this->get_logger(), "LivoxLidar callbacks set successfully.");
  // });
    while(!LivoxLidarSdkInit(mid360_config_path.c_str())){
      RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit failed, retry after 0.2s.");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      LivoxLidarSdkUninit();
    }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
  SetLivoxLidarImuDataCallback(ImuDataCallback, this);
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, this);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
  RCLCPP_INFO(this->get_logger(), "LivoxLidar callbacks set successfully.");

  // try{
  //   if(config["debug"]["mid360"].IsDefined()&&config["debug"]["mid360"].as<bool>()){
  //     DEBUGInit();
  //   }
  // }
  // catch(std::exception& e){
  //   RCLCPP_ERROR(this->get_logger(), "debug config not found. %s", e.what());
  // }

  RCLCPP_INFO(this->get_logger(), "Mid360Driver init successfully.");
}


Engineering_robot_RM2025_Pnx::Mid360Driver::~Mid360Driver() {
  LivoxLidarSdkUninit();
  RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkUninit successfully.");
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::PointCloudCallback(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  #ifdef cloudelog
  printf("PointCloudCallback called.");
  #endif
  if(data == nullptr) {
    return;
  }
  if(data->data_type == kLivoxLidarSphericalCoordinateData) {
     printf("data_type is kLivoxLidarSphericalCoordinateData not supported.");
    return;
  }

  LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZ> cloud;


  #ifdef cloudelog
  printf("""Received %d points.", data->dot_num);
  #endif

  for(int i = 0; i < data->dot_num; i++) {
    if(p_point_data[i].tag != 0) continue;
    cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
    #ifdef cloudelog
    printf("x:%f, y:%f, z:%f.",p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0);
    #endif
  }
  cloud.width=cloud.size();
  cloud.height = 1;
  cloud_msg.is_dense = true;

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
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  #endif
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (info == nullptr) {
       printf("lidar info change callback failed, the info is nullptr.\n");
      return;
  }
  else printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);

  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle; 
  std::stringstream ss; 
  ss << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: ";
  printf("%s", ss.str().c_str());
  printf("%s", info);
  return;
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (response == nullptr) {
      return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
        status, handle, response->ret_code, response->error_key);
}

void Engineering_robot_RM2025_Pnx::Mid360Driver::QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  Mid360Driver* node = static_cast<Mid360Driver*>(client_data);
  if (status != kLivoxLidarStatusSuccess) {
     printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, client_data);
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

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    Engineering_robot_RM2025_Pnx::Mid360Driver);

// ros2 run --prefix 'gdb -ex run --args'  sensordrivers mid360_driver_node --ros-args -p Location:="/home/pnx/code/Engineering_robot_RM2025_Pnx/install/interfaces/share/interfaces/../../../../" -p mid360_config_path:="/home/pnx/code/Engineering_robot_RM2025_Pnx/install/sensordrivers/share/sensordrivers/config/mid360_config.json"