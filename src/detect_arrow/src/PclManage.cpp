#include "DetectArrow.hpp"


void Arrow_detector::PointCloudeInit(){

    YAML::Node PCLManagerConfig=config["arrow_detect"]["PCLManager"];

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_,this);

    msgfillter_cloudpoint_sub.subscribe(this,"/sensor/mid360/point_cloud");
    msgfillter_image_sub.subscribe(this,"/sensor/image");

    sync_.reset(new Sync(SyncPolicy(10),msgfillter_cloudpoint_sub,msgfillter_image_sub));
    sync_->setMaxIntervalDuration(rclcpp::Duration(1,0));
    sync_->registerCallback(&Arrow_detector::ImageCloudPointCallBack,this);

    # ifdef test_pcl_manage

    pcl_test_point_cloud_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/onarrowcloud",10);

    # endif

    ransacDistanceThreshold=PCLManagerConfig["ransacDistanceThreshold"].as<double>();
    ransacMaxIterations=PCLManagerConfig["ransacMaxIterations"].as<int>();

    RCLCPP_INFO(this->get_logger(),"finish init PointCloudeInit");

}

void Arrow_detector::ImageCloudPointCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg){
    
    tf2::TimePoint image_time_point = tf2::TimePoint(std::chrono::seconds(image_msg->header.stamp.sec)+std::chrono::nanoseconds(image_msg->header.stamp.nanosec));
    tf2::TimePoint cloud_time_point = tf2::TimePoint(std::chrono::seconds(cloud_msg->header.stamp.sec)+std::chrono::nanoseconds(cloud_msg->header.stamp.nanosec));
    geometry_msgs::msg::TransformStamped transform;

    try{
        transform=tf2_buffer_->lookupTransform("sensor/mid360",
            cloud_time_point,
            "sensor/camera",
            image_time_point,
            "map");
    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(),"[ImageCloudPointCallBack]: %s",ex.what());
        return;
    }

    //get Image
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(image_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat originalframe=cv_ptr->image;
    originalframe.copyTo(OriginalImage_pcl);
    RCLCPP_INFO(this->get_logger(), "Get frame");

    // Target Arrow

    cv::Mat BinaryImage=this->PreProgress(OriginalImage_pcl);
    Counter2d CornerPoints=this->TargetArrow(BinaryImage);

    if(!CornerPoints.size()){
        RCLCPP_INFO(this->get_logger(),"target fail");
        return;
    }

    cv::Mat MaskCornerPoint(OriginalImage_pcl.size(),CV_8S);
    cv::Point2f CornerPointsCenter;
    float CornerPointsRadius;

    cv::minEnclosingCircle(CornerPoints, CornerPointsCenter, CornerPointsRadius);

    //preprocesse pointcloud

    sensor_msgs::msg::PointCloud2 TransformedCloudPoint;
    pcl::PointCloud<pcl::PointXYZ> CloudPointpcl;

    pcl::PointCloud<pcl::PointXYZ> CloudPointOnArrow;
    std::vector<Eigen::Matrix<double,3,1>> CloudPointImagePoint;
    
    tf2::doTransform(*cloud_msg,TransformedCloudPoint,transform);
    pcl::fromROSMsg<pcl::PointXYZ>(TransformedCloudPoint,CloudPointpcl);

    for(auto & i : CloudPointpcl){
        if(i.z<0) continue;
        Eigen::Matrix<double,4,1> cloudpointEigen;
        Eigen::Matrix<double,3,1> imagePoint;
        cloudpointEigen<<i.x, i.y, i.z, 1;

        imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
        imagePoint/=imagePoint(2);

        if(0<=imagePoint(0)&&imagePoint(0)<=1280&&
            0<=imagePoint(1)&&imagePoint(1)<=1080&&
            inCircle(CornerPointsCenter,
                CornerPointsRadius,
                imagePoint)
            ){
                CloudPointOnArrow.push_back(i);
                CloudPointImagePoint.push_back(std::move(imagePoint));
        }
    }

    # ifdef test_pcl_manage

    sensor_msgs::msg::PointCloud2 inarrowPointCloudRos;
    pcl::toROSMsg<pcl::PointXYZ>(CloudPointOnArrow,inarrowPointCloudRos);
    inarrowPointCloudRos.header.frame_id="sensor/camera";
    inarrowPointCloudRos.header.stamp=this->now();
    RCLCPP_INFO(this->get_logger(),"Before %ld After %ld",CloudPointpcl.size(),CloudPointOnArrow.size());
    pcl_test_point_cloud_pub->publish(inarrowPointCloudRos);

    # endif

    

}

bool Arrow_detector::GetTRvecPointCloud_PC(const pcl::PointCloud<pcl::PointXYZ> & pointcloud, Counter2d CornerPoints, cv::Mat & tvec, cv::Mat & rvec){
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pointcloud)));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ransacDistanceThreshold);
    ransac.setMaxIterations(ransacMaxIterations);
    ransac.computeModel();

    // useful index of pointcloud
    std::vector<int> inliers;
    // Ax+By+Cz+D=0
    Eigen::VectorXf coefficient;
    std::vector<cv::Point3d> Points3D;

    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coefficient);

    ImagePointTo3DPoint_Plant(CornerPoints,coefficient,Points3D);

    RCLCPP_INFO(this->get_logger(),"ImagePointTo3DPoint_Plant OK!");

    

}

bool Arrow_detector::ImagePointTo3DPoint_Plant(const Counter2d& Points2D, Eigen::VectorXf plant, std::vector<cv::Point3d> Points3D){
    Points3D.clear();

    std::vector<Eigen::Matrix<double,3,1>> Points3DnoZEigen;

    for(auto &i : Points2D){
        Eigen::Matrix<double,3,1> Point2dlin,Point3dnoZlin;
        Point2dlin<<i.x,i.y,1;
        Point3dnoZlin=InverseCameraMatrixEigen*Point2dlin;
        Point3dnoZlin/=Point3dnoZlin(2);
        Points3DnoZEigen.push_back(std::move(Point3dnoZlin));
    }

    for(auto & i : Points3DnoZEigen){
        double Z=CalculatePlantEquality(plant,std::vector<double>{i(0),i(1)},2);
        Points3D.push_back(cv::Point3d(i(0),i(1),Z));
    }

    return 0;
}

bool Arrow_detector::inCircle(const cv::Point2f & Center,
    const float & CornerPointsRadius,
    const Eigen::Matrix<double,3,1>& TestPoint){

    assert(CornerPointsRadius>0);

    float distance_2=((TestPoint(0)-Center.x)*(TestPoint(0)-Center.x)+
        (TestPoint(1)-Center.y)*(TestPoint(1)-Center.y));
    
    return distance_2<=CornerPointsRadius*CornerPointsRadius;
}

/*
接下来的思路：

我们发现以确定的坐标轴和原点得到准确坐标的路径有两个

1. 先拟合平面，然后再根据拟合出来的平面，算出得到的角点的空间位置
2. 直接通过选择的角点，得到的直线，从已知的点云中选择点来组成坐标系
3. 直接边缘拟合，得到边框的确切边缘，然后用数学关系直接求得兑换框中心点位置
*/