#include "DetectArrow.hpp"


void Arrow_detector::PointCloudeInit(){

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);

    msgfillter_cloudpoint_sub.subscribe(this,"/senser/mid360/point_cloud");
    msgfillter_image_sub.subscribe(this,"/sensor/image");

    sync_.reset(new Sync(SyncPolicy(10),msgfillter_cloudpoint_sub,msgfillter_image_sub));
    sync_->registerCallback(std::bind(&Arrow_detector::ImageCloudPointCallBack,this));
}