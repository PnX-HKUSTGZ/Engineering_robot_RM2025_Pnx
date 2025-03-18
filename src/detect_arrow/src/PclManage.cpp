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
        // transform=tf2_buffer_->lookupTransform(
        //     "sensor/mid360",
        //     cloud_time_point,
        //     "sensor/camera",
        //     image_time_point,
        //     "map");
        transform=tf2_buffer_->lookupTransform(
            "sensor/mid360",
            "sensor/camera",
            this->now()
            );

        #ifdef test_pcl_manage
        std::stringstream transformsss;
        transformsss<<"rotation: "<<
        transform.transform.rotation.x<<" "<<
        transform.transform.rotation.y<<" "<<
        transform.transform.rotation.z<<" "<<
        transform.transform.rotation.w<<"\n";
        transformsss<<"translation: "<<
        transform.transform.translation.x<<" "<<
        transform.transform.translation.y<<" "<<
        transform.transform.translation.z;
        RCLCPP_INFO(this->get_logger(),"transform %s",transformsss.str().c_str());
        #endif
    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(),"[ImageCloudPointCallBack]: %s",ex.what());
        return;
    }

    transform.transform.rotation.x=0;
    transform.transform.rotation.y=-0.7071068;
    transform.transform.rotation.z=0.7071068;
    transform.transform.rotation.w=0;
    transform.transform.translation.x=0.06623;
    // transform.transform.translation.y=-0.0333;
    transform.transform.translation.y=-0.03257;
    // transform.transform.translation.z=0.03257;
    transform.transform.translation.z=-0.0333;

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
    Counter2d CornerPoints=this->TargetArrow(BinaryImage,OriginalImage_pcl);

    if(!CornerPoints.size()){
        RCLCPP_INFO(this->get_logger(),"target fail");
        return;
    }

    cv::Mat MaskCornerPoint(OriginalImage_pcl.size(),CV_8S);
    // cv::Point2f CornerPointsCenter;
    // float CornerPointsRadius;
    cv::RotatedRect rotateRectCounterPoints;

    rotateRectCounterPoints=cv::minAreaRect(CornerPoints);

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
        # ifdef test_pcl_manage

        // RCLCPP_INFO(this->get_logger(),"CloudPointpcl to imagePoint Oricloud [%lf,%lf,%lf,%lf]",cloudpointEigen(0),cloudpointEigen(1),cloudpointEigen(2),cloudpointEigen(3));
        // RCLCPP_INFO(this->get_logger(),"CloudPointpcl to imagePoint transedcloud [%lf,%lf,%lf,%lf]",cloudpointEigen(0),cloudpointEigen(1),cloudpointEigen(2),cloudpointEigen(3));
        // RCLCPP_INFO(this->get_logger(),"CloudPointpcl to imagePoint imagePoint [%lf,%lf,%lf]",imagePoint(0),imagePoint(1),imagePoint(2));

        # endif
        imagePoint/=imagePoint(2);
        if(0<=imagePoint(0)&&imagePoint(0)<=1280&&
            0<=imagePoint(1)&&imagePoint(1)<=1080&&
            isPointInsideRotatedRect(cv::Point2f(imagePoint(0),imagePoint(1)),rotateRectCounterPoints)
            ){
                CloudPointOnArrow.push_back(i);
                CloudPointImagePoint.push_back(std::move(imagePoint));
        }
    }

    # ifdef test_pcl_manage

    for(auto i : CloudPointImagePoint){
        cv::circle(OriginalImage_pcl,cv::Point(i(0),i(1)),1,cv::Scalar(22,33,130),-1);
    }

    sensor_msgs::msg::PointCloud2 inarrowPointCloudRos;
    pcl::toROSMsg<pcl::PointXYZ>(CloudPointOnArrow,inarrowPointCloudRos);
    inarrowPointCloudRos.header.frame_id="sensor/camera";
    inarrowPointCloudRos.header.stamp=this->now();
    RCLCPP_INFO(this->get_logger(),"Before %ld After %ld",CloudPointpcl.size(),CloudPointOnArrow.size());
    TransformedCloudPoint.header.frame_id="sensor/camera";
    TransformedCloudPoint.header.stamp=this->now();
    pcl_test_point_cloud_pub->publish(inarrowPointCloudRos);

    # endif

    cv::Mat tvec,rvec;
    GetTRvecPointCloud_PC(CloudPointOnArrow,CornerPoints,tvec,rvec);

    DrawPnPResult(OriginalImage_pcl,rvec,tvec,cv::Scalar(225,80,22),3,cv::Point(20,40));

    cv::imshow("OriginalImage_pcl",OriginalImage_pcl);
    cv::waitKey(20);

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

    
    bool ImagePointTo3DPoint_PlantCheck=ImagePointTo3DPoint_Plant(CornerPoints,coefficient,Points3D);
    if(ImagePointTo3DPoint_PlantCheck){
        RCLCPP_WARN(this->get_logger(),"ImagePointTo3DPoint_PlantCheck fail");
        return 1;
    }

    # ifdef test_pcl_manage

    std::stringstream coefficientsss;
    coefficientsss<<"A :"<<coefficient(0)<<" ";
    coefficientsss<<"B :"<<coefficient(1)<<" ";
    coefficientsss<<"C :"<<coefficient(2)<<" ";
    coefficientsss<<"D :"<<coefficient(3)<<" ";

    RCLCPP_INFO(this->get_logger(),"coefficient %s",coefficientsss.str().c_str());
        
    std::srand(this->now().nanoseconds()%100000);
    std::vector<cv::Point3d> RandomTranglePoints;
    for(int i=0;i<3;i++){
        int x=rand(),y=rand();
        if(i==0) x=-0.01,y=-0.01;
        if(i==1) x=0.001,y=-0.001;
        if(i==2) x=0.001,y=0.001;

        cv::Point3d inputome=cv::Point3d(x,y,CalculatePlantEquality(coefficient,std::vector<double>{double(x),double(y)},2));
        RandomTranglePoints.push_back(inputome);

        if(std::abs(inputome.x*coefficient(0)+inputome.y*coefficient(1)+inputome.z*coefficient(2)+coefficient(3))>1e-9){
            RCLCPP_ERROR(this->get_logger(),"CalculatePlantEquality fail");
            cv::waitKey(0);
        }
    }

    std::vector<cv::Point2d> PlantImagePoints = Points3to2Transform(cameraMatrixEigen,RandomTranglePoints);

    for(auto &i : PlantImagePoints){
        RCLCPP_INFO(this->get_logger(),"PlantImagePoints %lf, %lf",i.x,i.y);
    }

    cv::drawContours(OriginalImage_pcl,Counters{[&](){
        Counter ans;
        for(auto & i : PlantImagePoints){
            ans.push_back(cv::Point(i.x,i.y));
        }
        return ans;
    }()},-1,cv::Scalar(22,130,90),-1);
        
    #endif


    bool KabschAlgorithmCheck=KabschAlgorithm(Points3D,objpoints,tvec,rvec);
    if(KabschAlgorithmCheck){
        RCLCPP_WARN(this->get_logger(),"KabschAlgorithm fail");
        return KabschAlgorithmCheck;
    }
    return 0;

}

bool Arrow_detector::ImagePointTo3DPoint_Plant(const Counter2d& Points2D, const Eigen::VectorXf & plant, std::vector<cv::Point3d> &Points3D){
    Points3D.clear();

    std::vector<Eigen::Matrix<double,3,1>> Points3DnoZEigen;

    for(auto &i : Points2D){
        Eigen::Matrix<double,3,1> Point2dlin,Point3dnoZlin;
        Point2dlin<<i.x,i.y,1;
        Point3dnoZlin=InverseCameraMatrixEigen*Point2dlin;
        Point3dnoZlin/=Point3dnoZlin(2);
        Points3DnoZEigen.push_back(std::move(Point3dnoZlin));
        // RCLCPP_INFO(this->get_logger(),"ImagePointTo3DPoint_Plant Count!");
    }
    RCLCPP_INFO(this->get_logger(),"Points3DnoZEigen size %ld",Points3DnoZEigen.size());

    for(auto & i : Points3DnoZEigen){
        double Z=CalculatePlantEquality(plant,std::vector<double>{i(0),i(1)},2);
        if(std::isnan(Z)){
            RCLCPP_WARN(this->get_logger(),"Points3DnoZEigen get nan");
            return 1;
        }
        Points3D.push_back(cv::Point3d(i(0),i(1),Z));
    }
    // RCLCPP_INFO(this->get_logger(),"Points3D size %ld",Points3D.size());

    RCLCPP_INFO(this->get_logger(),"ImagePointTo3DPoint_Plant OK!");
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