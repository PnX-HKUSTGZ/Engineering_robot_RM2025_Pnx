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
        transform=tf2_buffer_->lookupTransform(
            "sensor/camera",
            image_time_point,
            "sensor/mid360",
            cloud_time_point,
            "map"
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

    // transform.transform.rotation.x=0;
    // transform.transform.rotation.y=-0.7071068;
    // transform.transform.rotation.z=0.7071068;
    // transform.transform.rotation.w=0;
    // transform.transform.translation.x=0.06623;
    // // transform.transform.translation.y=-0.0333;
    // transform.transform.translation.y=-0.03257;
    // // transform.transform.translation.z=0.03257;
    // transform.transform.translation.z=-0.0333;

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
    Counter2f CornerPointsf=[&CornerPoints](){
        Counter2f ans;
        for(auto & I : CornerPoints){
            ans.push_back(cv::Point2f(I.x,I.y));
        }
        return ans;
    }();

    if(!CornerPoints.size()){
        RCLCPP_INFO(this->get_logger(),"target fail");
        return;
    }

    cv::Mat MaskCornerPoint(OriginalImage_pcl.size(),CV_8S);
    // cv::Point2f CornerPointsCenter;
    // float CornerPointsRadius;
    cv::RotatedRect rotateRectCounterPoints;

    // cv::minEnclosingCircle(CornerPointsf,CornerPointsCenter,CornerPointsRadius);

    rotateRectCounterPoints=cv::minAreaRect([&CornerPoints](){
        Counter2f ans;
        for(auto &i : CornerPoints){
            ans.push_back(cv::Point2f(i.x,i.y));
        }
        return ans;
    }());

    //preprocesse pointcloud

    sensor_msgs::msg::PointCloud2 TransformedCloudPoint;
    pcl::PointCloud<pcl::PointXYZ> CloudPointpcl;

    pcl::PointCloud<pcl::PointXYZ> CloudPointOnArrow;
    std::vector<Eigen::Matrix<double,3,1>> CloudPointImagePoint;
    
    tf2::doTransform(*cloud_msg,TransformedCloudPoint,transform);
    pcl::fromROSMsg<pcl::PointXYZ>(TransformedCloudPoint,CloudPointpcl);

    for(auto & i : CloudPointpcl){
        if(i.z<0||i.z>2) continue;
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
            // inCircle(CornerPointsCenter,CornerPointsRadius,imagePoint)
            ){
                CloudPointOnArrow.push_back(i);
                CloudPointImagePoint.push_back(std::move(imagePoint));
        }
    }

    # ifdef test_pcl_manage

    for(auto i : CloudPointImagePoint){
        cv::circle(OriginalImage_pcl,cv::Point(i(0),i(1)),1,cv::Scalar(22,33,130),-1);
    }

    DrawRotatedRect(OriginalImage_pcl,rotateRectCounterPoints,cv::Scalar(102,32,210),1);

    // sensor_msgs::msg::PointCloud2 inarrowPointCloudRos;
    // pcl::toROSMsg<pcl::PointXYZ>(CloudPointOnArrow,inarrowPointCloudRos);
    // inarrowPointCloudRos.header.frame_id="sensor/camera";
    // inarrowPointCloudRos.header.stamp=this->now();
    // RCLCPP_INFO(this->get_logger(),"Before %ld After %ld",CloudPointpcl.size(),CloudPointOnArrow.size());
    // TransformedCloudPoint.header.frame_id="sensor/camera";
    // TransformedCloudPoint.header.stamp=this->now();
    // pcl_test_point_cloud_pub->publish(inarrowPointCloudRos);

    # endif

    cv::Mat tvec,rvec;
    GetTRvecPointCloud_PC(CloudPointOnArrow,CornerPoints,tvec,rvec);

    cloudressMtx.lock();
    cloudress.push(PnPresult(tvec,rvec,this->now()));
    cloudressMtx.unlock();

    # ifdef test_pcl_manage
    DrawPnPResult(OriginalImage_pcl,rvec,tvec,cv::Scalar(225,80,22),3,cv::Point(20,40));

    cv::imshow("OriginalImage_pcl",OriginalImage_pcl);
    cv::waitKey(20);
    # endif

}

bool Arrow_detector::GetTRvecPointCloud_PC(const pcl::PointCloud<pcl::PointXYZ> & pointcloud, Counter2d CornerPoints, cv::Mat & tvec, cv::Mat & rvec){
    
    // pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(
    //     std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pointcloud)));
    // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    // ransac.setDistanceThreshold(ransacDistanceThreshold);
    // ransac.setMaxIterations(ransacMaxIterations);
    // ransac.computeModel();

    const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > pointcloudptr=std::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(pointcloud);
    pcl::PointCloud<pcl::PointXYZ> ExtractedPointCloud;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransacDistanceThreshold);
    seg.setMaxIterations(ransacMaxIterations);
    seg.setInputCloud(pointcloudptr);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointcloudptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(ExtractedPointCloud);

    // Ax+By+Cz+D=0
    Eigen::Vector4f coefficient=Eigen::Vector4f(coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2],
        coefficients->values[3]);
    // 储存了2D点映射得到的3D点
    std::vector<cv::Point3d> Points3D;
    

    
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

    for(auto &i : Points3D){
        std::stringstream Points3Dsss;
        Points3Dsss<<"Points3Dsss: "<<i.x<<" "<<i.y<<" "<<i.z;
        RCLCPP_INFO(this->get_logger(),"%s",Points3Dsss.str().c_str());
    }
    RCLCPP_INFO(this->get_logger(),"Points3Dsss end");

    RCLCPP_INFO(this->get_logger(),"coefficient %s",coefficientsss.str().c_str());
        
    std::srand(this->now().nanoseconds()%100000);
    std::vector<cv::Point3d> RandomTranglePoints;
    for(int i=0;i<8;i++){
        double z=-0.0001*i,y=1.25;
        if(i==0) y=-0.10698 ,z=1.19216;
        if(i==1) y=-0.107181 ,z=1.13963;
        if(i==2) y=-0.0200642,z= 0.845218;
        if(i==3) y=-0.0341553,z= 0.843324;
        if(i==4) y=-0.201081 ,z=0.820894;
        if(i==5) y=-0.186004 ,z=0.82292;
        if(i==6) y=-0.100522 ,z=1.16639;
        if(i==7) y=-0.113646 ,z=1.16586;
        
        

        double x=CalculatePlantEquality(coefficient,std::vector<double>{double(y),double(z)},0);

        if(std::isnan(y)) continue;

        RCLCPP_INFO(this->get_logger(),"Plant3DPoints %lf, %lf, %lf",x,y,z);
        cv::Point3d inputome=cv::Point3d(x,y,z);
        RandomTranglePoints.push_back(inputome);

    }

    std::vector<cv::Point2d> PlantImagePoints = Points3to2Transform(cameraMatrixEigen,RandomTranglePoints);

    for(auto &i : PlantImagePoints){
        RCLCPP_INFO(this->get_logger(),"PlantImagePoints %lf, %lf",i.x,i.y);
        cv::circle(OriginalImage_pcl,i,6,cv::Scalar(167,55,90),-1);
    }
        
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg<pcl::PointXYZ>(ExtractedPointCloud,msg);
    msg.header.frame_id="sensor/camera";
    msg.header.stamp=this->now();
    pcl_test_point_cloud_pub->publish(msg);

    for(auto i : ExtractedPointCloud){
        Eigen::Matrix<double,4,1>  ExtractedPointCloudEigen;
        ExtractedPointCloudEigen<<i.x,i.y,i.z,1;
        Eigen::Matrix<double,3,1> I=cameraMatrixEigen*signMat*ExtractedPointCloudEigen;
        I/=I(2);
        cv::circle(OriginalImage_pcl,cv::Point(I(0),I(1)),1,cv::Scalar(130,100,22),-1);
    }


    #endif


    bool KabschAlgorithmCheck=KabschAlgorithm(objpoints,Points3D,tvec,rvec);
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