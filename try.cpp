// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Vector3.h>
// #include <Eigen/Dense>

// class IMUOrientationEstimator {
// public:
//     IMUOrientationEstimator() {
//         ros::NodeHandle nh;
        
//         // 初始化四元数 (w, x, y, z)
//         q = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
//         last_time = ros::Time::now();
        
//         // 互补滤波参数
//         alpha = 0.98;  // 陀螺仪权重
        
//         // 订阅和发布
//         imu_sub = nh.subscribe("/imu/data", 10, &IMUOrientationEstimator::imuCallback, this);
//         ori_pub = nh.advertise<geometry_msgs::Quaternion>("/filtered_orientation", 10);
//         euler_pub = nh.advertise<geometry_msgs::Vector3>("/euler_angles", 10);
//     }

// private:
//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//         // 计算时间间隔
//         ros::Time current_time = msg->header.stamp;
//         double dt = (current_time - last_time).toSec();
//         last_time = current_time;

//         if (dt <= 0) return;

//         // 获取陀螺仪数据 (rad/s)
//         double gx = msg->angular_velocity.x;
//         double gy = msg->angular_velocity.y;
//         double gz = msg->angular_velocity.z;

//         // 四元数积分
//         updateQuaternion(gx, gy, gz, dt);

//         // 使用加速度计补偿
//         double ax = msg->linear_acceleration.x;
//         double ay = msg->linear_acceleration.y;
//         double az = msg->linear_acceleration.z;
        
//         complementaryFilter(ax, ay, az, dt);

//         // 发布结果
//         publishOrientation();
//         publishEulerAngles();
//     }

//     void updateQuaternion(double gx, double gy, double gz, double dt) {
//         // 四元数微分方程
//         Eigen::Vector4d omega(0, gx, gy, gz);
//         Eigen::Vector4d q_dot = 0.5 * quaternionMultiply(q, omega);
        
//         // 一阶积分
//         q += q_dot * dt;
//         normalizeQuaternion();
//     }

//     void complementaryFilter(double ax, double ay, double az, double dt) {
//         // 加速度计测量的重力向量
//         Eigen::Vector3d accel(ax, ay, az);
//         if (accel.norm() < 1e-6) return;
        
//         // 归一化加速度计测量值
//         accel.normalize();

//         // 从当前四元数计算预测重力方向
//         Eigen::Vector3d grav_pred(
//             2*(q[1]*q[3] - q[0]*q[2]),
//             2*(q[0]*q[1] + q[2]*q[3]),
//             q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
//         );

//         // 计算误差向量
//         Eigen::Vector3d error = accel.cross(grav_pred);

//         // 互补滤波校正陀螺仪偏差
//         Eigen::Vector3d correction = (1 - alpha) * error;
//         Eigen::Vector3d gyro_corrected(
//             gx + correction.x(),
//             gy + correction.y(),
//             gz + correction.z()
//         );

//         // 使用修正后的角速度更新四元数
//         updateQuaternion(gyro_corrected.x(), gyro_corrected.y(), gyro_corrected.z(), dt);
//     }

//     Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
//         return Eigen::Vector4d(
//             q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
//             q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
//             q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
//             q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
//         );
//     }

//     void normalizeQuaternion() {
//         q.normalize();
//     }

//     void publishOrientation() {
//         geometry_msgs::Quaternion q_msg;
//         q_msg.w = q[0];
//         q_msg.x = q[1];
//         q_msg.y = q[2];
//         q_msg.z = q[3];
//         ori_pub.publish(q_msg);
//     }

//     void publishEulerAngles() {
//         geometry_msgs::Vector3 euler;
        
//         // 四元数转欧拉角 (ZYX顺序)
//         double roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
//         double pitch = asin(2*(q[0]*q[2] - q[3]*q[1]));
//         double yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
        
//         euler.x = roll;
//         euler.y = pitch;
//         euler.z = yaw;
//         euler_pub.publish(euler);
//     }

//     ros::Subscriber imu_sub;
//     ros::Publisher ori_pub, euler_pub;
//     Eigen::Vector4d q;
//     ros::Time last_time;
//     double alpha;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "imu_orientation_estimator");
//     IMUOrientationEstimator estimator;
//     ros::spin();
//     return 0;
// }


