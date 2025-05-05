#include "SensorDrivers/RealSense.hpp"

namespace Engineering_robot_RM2025_Pnx{

    RealSense::RealSense(rclcpp::NodeOptions options):
        rclcpp::Node("RealSenseDriver",options){
        LoadParams();

        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("sensor/RealSense/image",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ image_pub_ ok !");

        pc_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/RealSense/point_cloud",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ init ok !");

        geometry_msgs::msg::TransformStamped image_to_center_msg;
        geometry_msgs::msg::TransformStamped depth_to_center_msg;

        rs2::config cfg_pointcloud;
        rs2::config cfg_image;
        // cfg.enable_stream(RS2_STREAM_DEPTH,640,360,RS2_FORMAT_Z16,30);
        // cfg.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_YUYV,30);
        cfg_pointcloud.enable_stream(RS2_STREAM_DEPTH,depth_wight,depth_hight,RS2_FORMAT_Z16);
        cfg_image.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_RGB8);

        try{
            pipe_pointcloud_=std::make_shared<rs2::pipeline>();
            pipe_image_=std::make_shared<rs2::pipeline>();
            pc_=std::make_shared<rs2::pointcloud>();
            auto pipline_profile_pc=pipe_pointcloud_->start(cfg_pointcloud);
            RCLCPP_INFO(this->get_logger(),"rs2 pipe point cloud start ok !");
            auto pipline_profile_image=pipe_image_->start(cfg_image);
            RCLCPP_INFO(this->get_logger(),"rs2 pipe image start ok !");

            auto depth_sensor = pipline_profile_pc.get_device().first<rs2::depth_sensor>();
            auto color_sensor = pipline_profile_image.get_device().first<rs2::color_sensor>();
    
            rs2::stream_profile depth_profile;
            bool depth_profile_ok=0;
            for (auto p : depth_sensor.get_stream_profiles()){
                if (p.stream_type() == RS2_STREAM_DEPTH) {
                    depth_profile = p;
                    depth_profile_ok=1;
                    // if(auto pf=p.as<rs2::video_stream_profile>()){
                    //     RCLCPP_INFO_STREAM(this->get_logger()," Stream config :");
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream name : "<<pf.stream_name());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream type : "<<pf.stream_type());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream fps : "<<pf.fps());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream resolution ratio ["<<pf.width()<<","<<pf.height()<<"]");
                    // }
                }
            }
    
            rs2::stream_profile color_profile;
            bool color_profile_ok=0;
            for (auto p : color_sensor.get_stream_profiles()){
                if (p.stream_type() == RS2_STREAM_COLOR){
                    color_profile = p;
                    color_profile_ok=1;
                    // if(auto pf=p.as<rs2::video_stream_profile>()){
                    //     RCLCPP_INFO_STREAM(this->get_logger()," Stream config :");
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream name : "<<pf.stream_name());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream type : "<<pf.stream_type());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream fps : "<<pf.fps());
                    //     RCLCPP_INFO_STREAM(this->get_logger()," stream resolution ratio ["<<pf.width()<<","<<pf.height()<<"]");
                    // }
                }

            }

            // get_config
            auto camera_config=pipline_profile_image.get_device().first<rs2::color_sensor>();

            RCLCPP_INFO_STREAM(this->get_logger(),"EXPOSURE TIME range"<<camera_config.get_option_range(RS2_OPTION_EXPOSURE));
            RCLCPP_INFO_STREAM(this->get_logger(),"GAIN range"<<camera_config.get_option_range(RS2_OPTION_GAIN));
            RCLCPP_INFO_STREAM(this->get_logger(),"BRIGHTNESS range"<<camera_config.get_option_range(RS2_OPTION_BRIGHTNESS));
            
            camera_config.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0.0f); // DISABLE_AUTO_EXPOSURE
            camera_config.set_option(RS2_OPTION_EXPOSURE,EXPOSURE);
            camera_config.set_option(RS2_OPTION_GAIN,GAIN);
            camera_config.set_option(RS2_OPTION_BRIGHTNESS,BRIGHTNESS);
            
            RCLCPP_INFO_STREAM(this->get_logger(),"set EXPOSURE TIME "<<camera_config.get_option(RS2_OPTION_EXPOSURE));
            RCLCPP_INFO_STREAM(this->get_logger(),"set GAIN "<<camera_config.get_option(RS2_OPTION_GAIN));
            RCLCPP_INFO_STREAM(this->get_logger(),"set BRIGHTNESS "<<camera_config.get_option(RS2_OPTION_BRIGHTNESS));

        }
        catch(const std::exception & e){
            RCLCPP_FATAL(this->get_logger(),"pipe launch fail with %s",e.what());
            rclcpp::shutdown();
        }

        tf2_static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(),"static_tf2 init ok !");

        if(this->get_parameter("USE_VITURAL_POSE").as_bool()){
            RCLCPP_INFO(this->get_logger(),"USE_VITURAL_POSE is true, use vitral link robot_base and realsense");


            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp=this->now();
            msg.header.frame_id="robot_base_link";
            msg.child_frame_id="sensor/RealSense";
            msg.transform.translation.x=0.142;
            msg.transform.translation.y=-0.0135;
            msg.transform.translation.z=0.3697;
            msg.transform.rotation.w=1;
            msg.transform.rotation.x=0;
            msg.transform.rotation.y=0;
            msg.transform.rotation.z=0;

            tf2_static_pub_->sendTransform(msg);

            RCLCPP_INFO(this->get_logger(),"send robot_base to RealSense static transform OK!");
        }

        depth_to_center_msg.header.frame_id="sensor/RealSense";
        depth_to_center_msg.child_frame_id="sensor/RealSense/depth";
        depth_to_center_msg.header.stamp=this->now();
        depth_to_center_msg.transform.translation.x=-0.02;
        depth_to_center_msg.transform.translation.y=-1.1*1e-3;
        depth_to_center_msg.transform.translation.z=0;
        depth_to_center_msg.transform.rotation.x=0.7071068;
        depth_to_center_msg.transform.rotation.y=0;
        depth_to_center_msg.transform.rotation.z=0;
        depth_to_center_msg.transform.rotation.w=-0.7071068;

        image_to_center_msg.header.frame_id="sensor/RealSense";
        image_to_center_msg.child_frame_id="sensor/RealSense/image";
        image_to_center_msg.header.stamp=this->now();
        image_to_center_msg.transform.translation.x=-0.035;
        image_to_center_msg.transform.translation.y=-1.1*1e-3;
        image_to_center_msg.transform.translation.z=0;
        image_to_center_msg.transform.rotation.x=0.7071068;
        image_to_center_msg.transform.rotation.y=0;
        image_to_center_msg.transform.rotation.z=0;
        image_to_center_msg.transform.rotation.w=-0.7071068;

        tf2_static_pub_->sendTransform(depth_to_center_msg);
        RCLCPP_INFO(this->get_logger(),"send RealSense to depthcenter static transform OK!");
        tf2_static_pub_->sendTransform(image_to_center_msg);
        RCLCPP_INFO(this->get_logger(),"send RealSense to imagecenter static transform OK!");

        point_cloud_thread_=std::make_shared<std::thread>([this](){
            while(1){
                auto start_time = std::chrono::steady_clock::now();
                this->RS_pc_pub_callback();
                auto end_time = std::chrono::steady_clock::now();
                auto duration = end_time - start_time;
                auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
                RCLCPP_INFO_STREAM(this->get_logger(),"One point cloud loop cost :"<<duration_ms.count() <<" ms");
            }
        });

        image_thread_=std::make_shared<std::thread>([this](){
            while(1){
                auto start_time = std::chrono::steady_clock::now();
                this->RS_image_pub_callback();
                auto end_time = std::chrono::steady_clock::now();
                auto duration = end_time - start_time;
                auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
                RCLCPP_INFO_STREAM(this->get_logger(),"One image loop cost :"<<duration_ms.count() <<" ms");
            }
        });

        RCLCPP_INFO(this->get_logger(),"Init OK!");

    }

    void RealSense::RS_pc_pub_callback(){
        auto frames = pipe_pointcloud_->wait_for_frames();
        
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
    }

    void RealSense::RS_image_pub_callback(){
        auto frames = pipe_image_->wait_for_frames();
        
        auto color = frames.get_color_frame();

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

    void RealSense::LoadParams(){
        this->declare_parameter<bool>("USE_VITURAL_POSE",true);
        this->declare_parameter<int>("depth_wight",1280);
        this->declare_parameter<int>("depth_hight",720);
        this->declare_parameter<double>("depmin",0.2);
        this->declare_parameter<double>("depmax",2);
        this->declare_parameter<double>("EXPOSURE",166);
        this->declare_parameter<double>("GAIN",10);
        this->declare_parameter<double>("BRIGHTNESS",0);

        depth_wight=this->get_parameter("depth_wight").as_int();
        depth_hight=this->get_parameter("depth_hight").as_int();
        depmax=this->get_parameter("depmax").as_double();
        depmin=this->get_parameter("depmin").as_double();
        EXPOSURE=this->get_parameter("EXPOSURE").as_double();
        GAIN=this->get_parameter("GAIN").as_double();
        BRIGHTNESS=this->get_parameter("BRIGHTNESS").as_double();
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

    std::ostream& operator<<(std::ostream& os, const rs2::option_range& range) {
        // 保存当前的流格式，以便在函数结束后恢复
        std::ios_base::fmtflags flags = os.flags();
        std::streamsize precision = os.precision();
    
        // 设置浮点数输出格式，例如固定小数点并设置精度
        os << std::fixed << std::setprecision(4); // 您可以根据需要调整精度
    
        // 将 option_range 的信息写入流
        os << "[min: " << range.min
           << ", max: " << range.max
           << ", step: " << range.step
           << ", default: " << range.def << "]";
    
        // 恢复原始的流格式
        os.flags(flags);
        os.precision(precision);
    
        // 返回流的引用，以便可以链式调用
        return os;
    }

}// Engineering_robot_RM2025_Pnx

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Engineering_robot_RM2025_Pnx::RealSense);