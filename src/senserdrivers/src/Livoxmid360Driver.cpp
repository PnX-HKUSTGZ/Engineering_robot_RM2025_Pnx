#include "SenserDrivers/Livoxmid360Driver.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

std::shared_ptr<Mid360Driver> node;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  #ifdef cloudelog
  RCLCPP_INFO(node->get_logger(),"PointCloudCallback called.");
  #endif
  // void*(dev_type);
  // void*(client_data);
  if(data == nullptr) {
    return;
  }
    // RCLCPP_INFO(rclcpp::get_logger("Mid360Driver:PointCloudCallback"),"point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
    //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  node->PublishPointCloud(data);
}

void Mid360Driver::PublishPointCloud(const LivoxLidarEthernetPacket* data) {
    if(data->data_type == kLivoxLidarSphericalCoordinateData) {
        RCLCPP_ERROR(this->get_logger(),"data_type is kLivoxLidarSphericalCoordinateData not supported.");
        return;
    }
    node->addPoint(data);
}

void Mid360Driver::PublishIMU(const LivoxLidarEthernetPacket* data){
  LivoxLidarImuRawPoint* imu_data=(LivoxLidarImuRawPoint*)data->data;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id="/sensor/mid360";
  imu_msg.header.stamp=node->get_clock()->now();
  imu_msg.angular_velocity.x=imu_data->acc_x;
  imu_msg.angular_velocity.y=imu_data->acc_y;
  imu_msg.angular_velocity.z=imu_data->acc_z;
  imu_msg.linear_acceleration.x=imu_data->gyro_x;
  imu_msg.linear_acceleration.y=imu_data->gyro_y;
  imu_msg.linear_acceleration.z=imu_data->gyro_z;
  this->imu_pub_->publish(imu_msg);
}

Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
  return Eigen::Vector4d(
      q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
      q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
      q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
      q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
  );
}


void Mid360Driver::synchronous_pose(sensor_msgs::msg::Imu::SharedPtr msg){
  // RCLCPP_INFO(this->get_logger(),"synchronous_pose called.");
  // rclcpp::Time now_time=this->get_clock()->now();
  // double dt= (now_time-msg->header.stamp).seconds();

  // this->update_pose_rotate(dt,msg);
  // this->update_pose_translate(dt,msg);
  // this->pub_pose(now_time);

}

void Mid360Driver::pub_pose(rclcpp::Time time){
  geometry_msgs::msg::TransformStamped t;

  YAML::Node mid360posconfig=config["object_pos"]["mid360"];

  t.header.stamp =time;
  t.header.frame_id = "map";
  t.child_frame_id = "sensor/mid360";
  t.transform.translation.x = mid360posconfig["translation"]["x"].as<double>();
  t.transform.translation.y = mid360posconfig["translation"]["y"].as<double>();
  t.transform.translation.z = mid360posconfig["translation"]["z"].as<double>();
  t.transform.rotation.x = mid360posconfig["rotate"]["x"].as<double>();
  t.transform.rotation.y = mid360posconfig["rotate"]["y"].as<double>();
  t.transform.rotation.z = mid360posconfig["rotate"]["z"].as<double>();
  t.transform.rotation.w = mid360posconfig["rotate"]["w"].as<double>();
  tf_broadcaster_->sendTransform(t);
  #ifdef cloudelog
  RCLCPP_INFO(this->get_logger(), "tf broadcaster successfully.");
  #endif
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    // void*(client_data);
    if (info == nullptr) {
        RCLCPP_ERROR(node->get_logger(),"lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    else RCLCPP_INFO(node->get_logger(),"LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data){
  // void*(client_data);
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

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data){
  if(data==nullptr) return;
  if(data->data_type!=kLivoxLidarImuData) return;
  node->PublishIMU(data);
}

void mid360_init(){
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  RCLCPP_INFO(node->get_logger(), "SetCallBack successfully.");
}

Mid360Driver::Mid360Driver(const rclcpp::NodeOptions & options)
: Node("mid360_driver", options) {
    // init pose
    this->declare_parameter<std::string>("Location","/home/pnx/code/Engineering_robot_RM2025_Pnx/");
    config=YAML::LoadFile(this->get_parameter("Location").as_string()+"/src/config.yaml");
    start_time=this->now();
    buffertime=config["mid_360"]["buffertime"].as<int>();

    if(config["camera"]){
      RCLCPP_INFO(this->get_logger(),"okk");
    }

    std::ifstream iff(this->get_parameter("Location").as_string()+"/src/config.yaml");
    configpath=this->get_parameter("Location").as_string()+"src/senserdrivers/config/mid360_config.json";
    RCLCPP_INFO(this->get_logger(),"mid360_configpath:%s",configpath.c_str());
    RCLCPP_INFO(this->get_logger(),"Location config : %s",(this->get_parameter("Location").as_string()+"/src/config.yaml").c_str());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;

    
    t.header.stamp =this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "sensor/mid360";
    t.transform.translation.x = config["object_pos"]["mid360"]["translation"]["x"].as<double>();
    t.transform.translation.y = config["object_pos"]["mid360"]["translation"]["y"].as<double>();
    t.transform.translation.z = config["object_pos"]["mid360"]["translation"]["z"].as<double>();
    t.transform.rotation.x = config["object_pos"]["mid360"]["rotate"]["x"].as<double>();
    t.transform.rotation.y = config["object_pos"]["mid360"]["rotate"]["y"].as<double>();
    t.transform.rotation.z = config["object_pos"]["mid360"]["rotate"]["z"].as<double>();
    t.transform.rotation.w = config["object_pos"]["mid360"]["rotate"]["w"].as<double>();
    tf_static_transform_broadcaster->sendTransform(t);

    RCLCPP_INFO(this->get_logger(), "tf broadcaster successfully.");

    while(!LivoxLidarSdkInit(configpath.c_str())){
        RCLCPP_ERROR(this->get_logger(), "LivoxLidarSdkInit failed. try again");
        LivoxLidarSdkUninit();
    }
    RCLCPP_INFO(this->get_logger(), "LivoxLidarSdkInit successfully.");

    cloud_buffer_timer_=this->create_wall_timer(std::chrono::milliseconds(buffertime),[&](){
        publishCloud();
    });

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/mid360/point_cloud", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor/mid360/imu", 10);
    frame_id="sensor/mid360";

}

void Mid360Driver::addPoint(const LivoxLidarEthernetPacket* data) {
  if((this->now()-start_time)>rclcpp::Duration::from_seconds(buffertime/1000.0)){
    std::stringstream sss;
    sss<<"now: "<<this->now().seconds()<<" "<<this->now().nanoseconds()<<
      " start_time "<<start_time.seconds()<<" "<<start_time.nanoseconds()<<"  "
      <<"buffertime "<<rclcpp::Duration::from_seconds(buffertime/1000).seconds()<<" "<<rclcpp::Duration::from_seconds(buffertime/1000).nanoseconds()
      <<" (this->now()-start_time) "<<(this->now()-start_time).seconds()<<" "<<(this->now()-start_time).nanoseconds();
      #ifdef cloudelog
      RCLCPP_INFO(this->get_logger(),"addPoint time out, this call will not publish cloud. time %s",sss.str().c_str());
      #endif
      return;
  }
  else {
      #ifdef cloudelog
      RCLCPP_INFO(this->get_logger(),"addPoint time in, this call will publish cloud. size: %ld",data->dot_num);
      #endif
  }
  LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
  cloudmtx.lock();
  for(size_t i=0;i<data->dot_num;i++){
      if(p_point_data[i].tag != 0) continue;
      cloud.push_back(pcl::PointXYZ(p_point_data[i].x/1000.0,p_point_data[i].y/1000.0,p_point_data[i].z/1000.0));
      // RCLCPP_INFO(node->get_logger(),"point: %f, %f, %f, %f",cloud.points.back().x,cloud.points.back().y,cloud.points.back().z);
  }
  cloud.width=cloud.size();
  cloud.height=1;
  cloud.is_dense=true;
  cloudmtx.unlock();
  #ifdef cloudelog
  RCLCPP_INFO(this->get_logger(),"addPoint successfully.");
  #endif
}

void Mid360Driver::publishCloud(builtin_interfaces::msg::Time time_) {
  cloudmtx.lock();
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud,cloud_msg);
  cloud_msg.header.frame_id=frame_id;
  cloud_msg.header.stamp=time_;
  point_cloud_pub_->publish(cloud_msg);
  cloudmtx.unlock();
  #ifdef cloudelog
  RCLCPP_INFO(this->get_logger(),"publishCloud successfully. size: %ld",cloud.size());
  #endif
  this->reset();
}

void Mid360Driver::reset() {
  // mtx_cloud.lock();
  #ifdef cloudelog
  RCLCPP_INFO(this->get_logger(),"reset cloud buffer.");
  #endif
  cloudmtx.lock();
  start_time=this->now();
  cloud.clear();
  cloudmtx.unlock();
  // mtx_cloud.unlock();
}

bool Mid360Driver::isoutoftime() {
  // mtx_cloud.lock();
  bool res=this->now()-start_time>rclcpp::Duration::from_seconds(buffertime/1000);
  // mtx_cloud.unlock();
  return res;
}

int main (int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    node = std::make_shared<Mid360Driver>();
    mid360_init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
