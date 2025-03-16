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
  if (data == nullptr) {
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
    // LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // sensor_msgs::msg::PointCloud2 cloud_msg;
    // int cloud_siz=0;
    // for(size_t i=0;i<data->dot_num;i++) cloud_siz += (p_point_data[i].tag == 0);
    // cloud.height=1;
    // cloud.width=cloud_siz;
    // cloud.is_dense=true;
    // cloud.points.resize(cloud.height*cloud.width);

    // for(size_t i=0,j=0;i<data->dot_num;i++){
    //     if(p_point_data[i].tag == 0){
    //         cloud.points[j].x=p_point_data[i].x/1000.0;
    //         cloud.points[j].y=p_point_data[i].y/1000.0;
    //         cloud.points[j].z=p_point_data[i].z/1000.0;
    //         RCLCPP_INFO(this->get_logger(),"point: %f, %f, %f, %f",cloud.points[j].x,cloud.points[j].y,cloud.points[j].z);
    //         j++;
    //     }
    // }
    // pcl::toROSMsg(cloud,cloud_msg);
    // cloud_msg.header.frame_id="/sensor/mid360";
    // cloud_msg.header.stamp=node->get_clock()->now();
    // point_cloud_pub_->publish(cloud_msg);
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

void Mid360Driver::update_pose_rotate(double dt,sensor_msgs::msg::Imu::SharedPtr msg){
  double gx=msg->angular_velocity.x;
  double gy=msg->angular_velocity.y;
  double gz=msg->angular_velocity.z;
  double ax=msg->linear_acceleration.x*g;
  double ay=msg->linear_acceleration.y*g;
  double az=msg->linear_acceleration.z*g;
  Eigen::Vector3d accel(ax,ay,az);
  Eigen::Vector4d omega_gyro(0,gx,gy,gz);
  Eigen::Vector4d q_dot = 0.5 * quaternionMultiply(this->pose_rotate,omega_gyro);
  // 一阶积分
  this->pose_rotate = this->pose_rotate + q_dot * dt;
  this->pose_rotate.normalize();

  if (accel.norm() < 1e-6) return;

  accel.normalize();
  Eigen::Vector3d grav_pred(
    2*(this->pose_rotate[1]*this->pose_rotate[3] - this->pose_rotate[0]*this->pose_rotate[2]),
    2*(this->pose_rotate[0]*this->pose_rotate[1] + this->pose_rotate[2]*this->pose_rotate[3]),
    this->pose_rotate[0]*this->pose_rotate[0] - this->pose_rotate[1]*this->pose_rotate[1] - this->pose_rotate[2]*this->pose_rotate[2] + this->pose_rotate[3]*this->pose_rotate[3]
  );

  Eigen::Vector3d error = accel.cross(grav_pred);

  Eigen::Vector3d correction = (1 - this->alpha) * error;
  Eigen::Vector3d gyro_corrected(
    gx + correction.x(),
    gy + correction.y(),
    gz + correction.z()
  );

  Eigen::Vector4d omega_accl = Eigen::Vector4d(0,gyro_corrected.x(),gyro_corrected.y(),gyro_corrected.z());
  q_dot = 0.5 * quaternionMultiply(this->pose_rotate,omega_accl);

  this->pose_rotate = this->pose_rotate + q_dot * dt;

  this->pose_rotate(1) /= this->pose_rotate(0);
  this->pose_rotate(2) /= this->pose_rotate(0);
  this->pose_rotate(3) /= this->pose_rotate(0);
  this->pose_rotate(0) = 1;
}

void Mid360Driver::update_pose_translate(double dt,sensor_msgs::msg::Imu::SharedPtr msg){
  Eigen::Vector3d acc_body=Eigen::Vector3d(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
  Eigen::Vector3d acc_world=Eigen::Quaterniond(this->pose_rotate)*acc_body;
  //减去重力
  RCLCPP_INFO(this->get_logger(),"acc_body: %f, %f, %f",acc_body(0),acc_body(1),acc_body(2));
  acc_body(2)=acc_body(2)-1;
  acc_body *=this->g;
  this->speed_translate=this->speed_translate+acc_world*dt;
  this->pose_translate=this->pose_translate+this->speed_translate*dt;
}


void Mid360Driver::synchronous_pose(sensor_msgs::msg::Imu::SharedPtr msg){
  // RCLCPP_INFO(this->get_logger(),"synchronous_pose called.");
  rclcpp::Time now_time=this->get_clock()->now();
  // double dt= (now_time-msg->header.stamp).seconds();

  // this->update_pose_rotate(dt,msg);
  // this->update_pose_translate(dt,msg);
  this->pub_pose(now_time);

}

void Mid360Driver::pub_pose(rclcpp::Time time){
  YAML::Node config(this->get_parameter("Location").as_string()+"/src/config.yaml");
  geometry_msgs::msg::TransformStamped t;

  YAML::Node mid360config=config["object_pos"]["mid360"];

  t.header.stamp =time;
  t.header.frame_id = "map";
  t.child_frame_id = "sensor/mid360";
  t.transform.translation.x = mid360config["translation"]["x"].as<double>();
  t.transform.translation.y = mid360config["translation"]["y"].as<double>();
  t.transform.translation.z = mid360config["translation"]["z"].as<double>();
  t.transform.rotation.x = mid360config["rotate"]["x"].as<double>();
  t.transform.rotation.y = mid360config["rotate"]["y"].as<double>();
  t.transform.rotation.z = mid360config["rotate"]["z"].as<double>();
  t.transform.rotation.w = mid360config["rotate"]["w"].as<double>();
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
  #ifdef cloudelog
  RCLCPP_INFO(rclcpp::get_logger("Mid360Driver:PointCloudCallback"),"ImuDataCallback called. imu handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n"
    ,handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  #endif
  node->PublishIMU(data);
}

void mid360_init(){
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  RCLCPP_INFO(node->get_logger(), "SetCallBack successfully.");
}

int main (int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    node = std::make_shared<Mid360Driver>();
    mid360_init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
