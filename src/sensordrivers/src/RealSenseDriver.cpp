#include "SensorDrivers/RealSense.hpp"

namespace Engineering_robot_RM2025_Pnx{

    RealSense::RealSense(rclcpp::NodeOptions options):
        rclcpp::Node("RealSenseDriver",options){

        this->declare_parameter<bool>("USE_VITURAL_POSE",true);
        if(!this->has_parameter("depth_wight")){
            RCLCPP_WARN(this->get_logger(),"not set depth_wight, use default 1280");
            this->declare_parameter<int>("depth_wight",1280);
        }
        if(!this->has_parameter("depth_hight")){
            RCLCPP_WARN(this->get_logger(),"not set depth_hight, use default 720");
            this->declare_parameter<int>("depth_hight",720);
        }
        if(!this->has_parameter("depth_hight")){
            RCLCPP_WARN(this->get_logger(),"not set depth_hight, use default 720");
            this->declare_parameter<int>("depth_hight",720);
        }
        if(!this->has_parameter("depmin")){
            RCLCPP_WARN(this->get_logger(),"not set depmin, use default 0.2");
            this->declare_parameter<double>("depmin",0.2);
        }
        if(!this->has_parameter("depmax")){
            RCLCPP_WARN(this->get_logger(),"not set depmax, use default 2");
            this->declare_parameter<double>("depmax",2);
        }
        depth_wight=this->get_parameter("depth_wight").as_int();
        depth_hight=this->get_parameter("depth_hight").as_int();
        depmax=this->get_parameter("depmax").as_double();
        depmin=this->get_parameter("depmin").as_double();

        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("sensor/RealSense/image",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ image_pub_ ok !");

        pc_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/RealSense/point_cloud",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ init ok !");


        geometry_msgs::msg::TransformStamped image_to_depth_msg;
        geometry_msgs::msg::TransformStamped depth_to_center_msg;

        rs2::config cfg;
        // cfg.enable_stream(RS2_STREAM_DEPTH,640,360,RS2_FORMAT_Z16,30);
        // cfg.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_YUYV,30);
        cfg.enable_stream(RS2_STREAM_DEPTH,depth_wight,depth_hight);
        cfg.enable_stream(RS2_STREAM_COLOR,1280,720);

        try{
            pipe_=std::make_shared<rs2::pipeline>();
            pc_=std::make_shared<rs2::pointcloud>();
            auto pipline_profile=pipe_->start(cfg);
            RCLCPP_INFO(this->get_logger(),"rs2 init and start ok !");

            auto depth_sensor = pipline_profile.get_device().first<rs2::depth_sensor>();
            auto color_sensor = pipline_profile.get_device().first<rs2::color_sensor>();
    
            rs2::stream_profile depth_profile;
            bool depth_profile_ok=0;
            for (auto p : depth_sensor.get_stream_profiles()){
                if (p.stream_type() == RS2_STREAM_DEPTH) {
                    depth_profile = p;
                    depth_profile_ok=1;
                    if(auto pf=p.as<rs2::video_stream_profile>()){
                        RCLCPP_INFO_STREAM(this->get_logger()," Stream config :");
                        RCLCPP_INFO_STREAM(this->get_logger()," stream name : "<<pf.stream_name());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream type : "<<pf.stream_type());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream fps : "<<pf.fps());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream resolution ratio ["<<pf.width()<<","<<pf.height()<<"]");
                    }
                    break;
                }
            }
    
            rs2::stream_profile color_profile;
            bool color_profile_ok=0;
            for (auto p : color_sensor.get_stream_profiles()){
                if (p.stream_type() == RS2_STREAM_COLOR){
                    color_profile = p;
                    color_profile_ok=1;
                    if(auto pf=p.as<rs2::video_stream_profile>()){
                        RCLCPP_INFO_STREAM(this->get_logger()," Stream config :");
                        RCLCPP_INFO_STREAM(this->get_logger()," stream name : "<<pf.stream_name());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream type : "<<pf.stream_type());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream fps : "<<pf.fps());
                        RCLCPP_INFO_STREAM(this->get_logger()," stream resolution ratio ["<<pf.width()<<","<<pf.height()<<"]");
                    }
                    break;
                }

            }
    
            if (!depth_profile_ok || !color_profile_ok) {
                RCLCPP_FATAL(this->get_logger(),"fail to find the depth or color profile");
            }

            rs2_extrinsics color_to_depth=color_profile.get_extrinsics_to(depth_profile);

            auto qua=rotationMatrixToQuaternion(color_to_depth.rotation);

            RCLCPP_INFO(this->get_logger(),"color to depth quaternion [x,y,z,w] : [%lf,%lf,%lf,%lf]",qua[0],qua[1],qua[2],qua[3]);
            RCLCPP_INFO(this->get_logger(),"color to depth translation [x,y,z] : [%f,%f,%f]",color_to_depth.translation[0],color_to_depth.translation[1],color_to_depth.translation[2]);

            image_to_depth_msg.header.frame_id="sensor/RealSense/depth";
            image_to_depth_msg.child_frame_id="sensor/RealSense/image";
            image_to_depth_msg.header.stamp=this->now();
            image_to_depth_msg.transform.rotation.x=qua[0];
            image_to_depth_msg.transform.rotation.y=qua[1];
            image_to_depth_msg.transform.rotation.z=qua[2];
            image_to_depth_msg.transform.rotation.w=qua[3];
            // color_to_depth.translation in meters
            image_to_depth_msg.transform.translation.x=color_to_depth.translation[0];
            image_to_depth_msg.transform.translation.y=color_to_depth.translation[1];
            image_to_depth_msg.transform.translation.z=color_to_depth.translation[2];

            
        }
        catch(const std::exception & e){
            RCLCPP_FATAL(this->get_logger(),"pipe launch fail with %s",e.what());
        }

        tf2_static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(),"static_tf2 init ok !");

        if(this->get_parameter("USE_VITURAL_POSE").as_bool()){
            RCLCPP_INFO(this->get_logger(),"USE_VITURAL_POSE is true, use vitral link robot_base and realsense");


            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp=this->now();
            msg.header.frame_id="robot_base";
            msg.child_frame_id="sensor/RealSense";
            msg.transform.rotation.w=1;
            msg.transform.translation.x=0;
            msg.transform.translation.y=0;
            msg.transform.translation.z=0;

            tf2_static_pub_->sendTransform(msg);

            RCLCPP_INFO(this->get_logger(),"send robot_base to RealSense static transform OK!");
        }

        depth_to_center_msg.header.frame_id="sensor/RealSense";
        depth_to_center_msg.child_frame_id="sensor/RealSense/depth";
        depth_to_center_msg.header.stamp=this->now();
        depth_to_center_msg.transform.translation.x=0.02;
        depth_to_center_msg.transform.translation.y=-1.1*1e-3;
        depth_to_center_msg.transform.translation.z=0;
        depth_to_center_msg.transform.rotation.x=0.7071068;
        depth_to_center_msg.transform.rotation.y=0;
        depth_to_center_msg.transform.rotation.z=0;
        depth_to_center_msg.transform.rotation.w=-0.7071068;

        tf2_static_pub_->sendTransform(depth_to_center_msg);
        RCLCPP_INFO(this->get_logger(),"send RealSense to depthcenter static transform OK!");
        tf2_static_pub_->sendTransform(image_to_depth_msg);
        RCLCPP_INFO(this->get_logger(),"send depthcenter to imagecenter static transform OK!");

        deal_pipe_thread_=std::make_shared<std::thread>([this](){
            while(1){
                auto start_time = std::chrono::steady_clock::now();
                this->RS_image_pc_pub_callback();
                auto end_time = std::chrono::steady_clock::now();
                auto duration = end_time - start_time;
                auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
                RCLCPP_INFO_STREAM(this->get_logger(),"One loop cost :"<<duration_ms.count() <<" ms");
            }
        });

        RCLCPP_INFO(this->get_logger(),"Init OK!");

    }

    void RealSense::RS_image_pc_pub_callback(){
        auto frames = pipe_->wait_for_frames();
        
        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();

        points=pc_->calculate(depth);

        auto pcl_point=points_to_pcl(points);

        auto pcl_point_filtered=filterDepthRange(pcl_point,depmin,depmax);

        sensor_msgs::msg::PointCloud2 pointcloudmsg;

        pcl::toROSMsg(*pcl_point_filtered,pointcloudmsg);

        pointcloudmsg.header.stamp=this->now();
        pointcloudmsg.header.frame_id="sensor/RealSense/depth";
        
        pc_pub_->publish(pointcloudmsg);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ publish ok! with point size : %ld",pcl_point_filtered->size());

        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        cv::Mat imagerbg(cv::Size(w,h),CV_8UC3, (void*)color.get_data(),cv::Mat::AUTO_STEP);
        // cv::Mat imagebgr;
        // cv::cvtColor(imagerbg,imagebgr,cv::COLOR_RGB2BGR);
        
        auto image_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),"rgb8",imagerbg).toImageMsg();
        image_ptr->header.frame_id="sensor/RealSense/image";
        image_ptr->header.stamp=this->now();

        image_pub_->publish(*image_ptr);
        RCLCPP_INFO(this->get_logger(),"image_pub_ publish ok!");

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto& p : cloud->points){
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }
    
        return cloud;
    }

    std::vector<double> rotationMatrixToQuaternion(const float matrix[9]){
        std::vector<double> mat;
        for(int i=0;i<9;i++) mat.push_back(matrix[i]);
        return rotationMatrixToQuaternion(mat);
    }

    std::vector<double> rotationMatrixToQuaternion(const std::vector<double> & matrix) {
        assert(matrix.size()==9);
        std::vector<double> q(4); // 四元数存储为 [x, y, z, w]
    
        // 提取矩阵元素以便阅读
        double R00 = matrix[0], R01 = matrix[1], R02 = matrix[2];
        double R10 = matrix[3], R11 = matrix[4], R12 = matrix[5];
        double R20 = matrix[6], R21 = matrix[7], R22 = matrix[8];
    
        // 计算矩阵的迹 (Trace)
        double trace = R00 + R11 + R22;
    
        if (trace > 0) {
            double s = std::sqrt(trace + 1.0) * 2; // S = 4*qw
            q[3] = 0.25 * s; // w
            q[0] = (R21 - R12) / s; // x
            q[1] = (R02 - R20) / s; // y
            q[2] = (R10 - R01) / s; // z
        } else {
            // 迹小于等于 0 时，找出对角线元素中最大的那个
            if (R00 > R11 && R00 > R22) {
                double s = std::sqrt(R00 - R11 - R22 + 1.0) * 2; // S = 4*qx
                q[0] = 0.25 * s; // x
                q[3] = (R21 - R12) / s; // w
                q[1] = (R01 + R10) / s; // y
                q[2] = (R02 + R20) / s; // z
            } else if (R11 > R22) {
                double s = std::sqrt(R11 - R00 - R22 + 1.0) * 2; // S = 4*qy
                q[1] = 0.25 * s; // y
                q[3] = (R02 - R20) / s; // w
                q[0] = (R01 + R10) / s; // x
                q[2] = (R12 + R21) / s; // z
            } else {
                double s = std::sqrt(R22 - R00 - R11 + 1.0) * 2; // S = 4*qz
                q[2] = 0.25 * s; // z
                q[3] = (R10 - R01) / s; // w
                q[0] = (R02 + R20) / s; // x
                q[1] = (R12 + R21) / s; // y
            }
        }
    
        double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;    
        return q;
    }

    /**
     * @brief 使用 PCL PassThrough 过滤器过滤深度在指定范围之外的点云
     *
     * @param input_cloud 输入点云智能指针
     * @param depmin 允许通过的最小深度值 (Z轴), 单位与点云一致
     * @param depmax 允许通过的最大深度值 (Z轴), 单位与点云一致
     * @return 过滤后的点云智能指针
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterDepthRange(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        float depmin,
        float depmax){
        // 创建一个 PassThrough 过滤器对象
        pcl::PassThrough<pcl::PointXYZ> pass;

        // 设置输入点云
        pass.setInputCloud(input_cloud);

        // 设置要过滤的轴 (Z轴表示深度)
        pass.setFilterFieldName("z");

        // 设置允许通过的范围 [depmin, depmax]
        pass.setFilterLimits(depmin, depmax);

        pass.setNegative(false); // 保留范围内的点，移除范围外的点

        // 创建用于存储过滤后点云的对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 应用过滤器
        pass.filter(*filtered_cloud);

        return filtered_cloud;
    }


}// Engineering_robot_RM2025_Pnx

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Engineering_robot_RM2025_Pnx::RealSense);