#include "SensorDrivers/RealSense.hpp"

namespace Engineering_robot_RM2025_Pnx{

    RealSense::RealSense(rclcpp::NodeOptions options):
        rclcpp::Node("RealSenseDriver",options){

        this->declare_parameter<bool>("USE_VITURAL_POSE",true);

        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("sensor/RealSense/image",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ image_pub_ ok !");

        pc_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor/RealSense/point_cloud",10);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ init ok !");

        pipe_=std::make_shared<rs2::pipeline>();
        pc_=std::make_shared<rs2::pointcloud>();
        pipe_->start();
        RCLCPP_INFO(this->get_logger(),"rs2 init and start ok !");

        if(this->get_parameter("USE_VITURAL_POSE").as_bool()){
            RCLCPP_INFO(this->get_logger(),"USE_VITURAL_POSE is true, use vitral link robot_base and realsense");
            tf2_static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            RCLCPP_INFO(this->get_logger(),"static_tf2 init ok !");

            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp=this->now();
            msg.header.frame_id="robot_base";
            msg.child_frame_id="sensor/RealSense";
            msg.transform.rotation.w=1;
            msg.transform.translation.x=0;
            msg.transform.translation.y=0;
            msg.transform.translation.z=0;

            tf2_static_pub_->sendTransform(msg);

            RCLCPP_INFO(this->get_logger(),"send static transform OK!");
        }

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

        sensor_msgs::msg::PointCloud2 pointcloudmsg;

        pcl::toROSMsg(*pcl_point,pointcloudmsg);

        pointcloudmsg.header.stamp=this->now();
        pointcloudmsg.header.frame_id="sensor/RealSense";
        
        pc_pub_->publish(pointcloudmsg);
        RCLCPP_INFO(this->get_logger(),"pc_pub_ publish ok! with point size : %ld",pcl_point->size());

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

}// Engineering_robot_RM2025_Pnx

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Engineering_robot_RM2025_Pnx::RealSense);