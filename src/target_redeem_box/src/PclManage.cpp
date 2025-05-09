#include "target_redeem_box/RedeemBox_detector.hpp"

namespace Engineering_robot_RM2025_Pnx{

void RedeemBox_detector::PointCloudeInit(){

    YAML::Node PCLManagerConfig=config["RedeemBox_detector"]["Parameters"]["PCLManager"];

    // msgfillter_cloudpoint_sub.subscribe(this,"/sensor/mid360/point_cloud");
    // msgfillter_image_sub.subscribe(this,"/sensor/image");

    // sync_.reset(new Sync(SyncPolicy(10),msgfillter_cloudpoint_sub,msgfillter_image_sub));
    // sync_->setMaxIntervalDuration(rclcpp::Duration(1,0));
    // sync_->registerCallback(&RedeemBox_detector::ImageCloudPointCallBack,this);

    # ifdef test_pcl_manage

    pcl_test_point_cloud_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/onarrowcloud",10);
    pcl_camera_point_cloud_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/camerapcl",10);

    # endif

    ransacDistanceThreshold=PCLManagerConfig["ransacDistanceThreshold"].as<double>();
    ransacMaxIterations=PCLManagerConfig["ransacMaxIterations"].as<int>();
    ransacMinInliersNum=PCLManagerConfig["ransacMinInliersNum"].as<int>();
    ransacMaxPlaneNum=PCLManagerConfig["ransacMaxPlaneNum"].as<int>();
    CloseThresehold=PCLManagerConfig["CloseThresehold"].as<double>();
    PlaneOnCornersNumThreshold=PCLManagerConfig["PlaneOnCornersNumThreshold"].as<int>();
    minPlaneDisThreshold=PCLManagerConfig["minPlaneDisThreshold"].as<double>();

    RCLCPP_INFO(this->get_logger(),"finish init PointCloudeInit");

}

bool RedeemBox_detector::GetTRvecPointCloud_PC(const pcl::PointCloud<pcl::PointXYZ> & inputPointCloudWiderROI, Counter2d CornerPoints, cv::Mat & tvec, cv::Mat & rvec){
    if (inputPointCloudWiderROI.empty()) {
        RCLCPP_WARN(this->get_logger(), "Input point cloud is empty");
        return 0;
    }
    if (CornerPoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "No corner points provided");
        return 0;
    }

    auto ExtractedPlanes = segmentPlanesWithPoints(
        std::make_shared<pcl::PointCloud<pcl::PointXYZ> >(inputPointCloudWiderROI),
        ransacDistanceThreshold,
        ransacMaxIterations,
        ransacMinInliersNum,
        ransacMaxPlaneNum);

    if(!ExtractedPlanes.size()){
        RCLCPP_ERROR(this->get_logger(),"segmentPlanesWithPoints fail to get any plane!");
        return 0;
    }
    
    // choose the best plant

    cv::Mat pnprvec,pnptvec;

    bool solvePnPcheck=cv::solvePnP(objpoints,CornerPoints,cameraMatrixMat,std::vector<double>{0,0,0,0,0},pnprvec,pnptvec,0,cv::SOLVEPNP_IPPE);

    DrawPnPResult(OriginalImage_pcl,pnprvec,pnptvec,cv::Scalar(33,223,123),1,cv::Point(-312312,-31231));

    if(!solvePnPcheck){
        RCLCPP_ERROR(this->get_logger(),"[GetTRvecPointCloud_PC] PnP failed!");
        return 0;
    }

    std::vector<double> PNPPlane;

    // {// get plane

    // cv::Mat rmat;
    // cv::Rodrigues(pnprvec,rmat);
    // Eigen::Matrix<double,4,4> rtvecEigen;

    // for(int i=0;i<3;i++){
    //     for(int e=0;e<3;e++){
    //         rtvecEigen(i,e)=rmat.at<double>(i,e);
    //     }
    //     rtvecEigen(i,3)=pnptvec.at<double>(i);
    // }
    // for(int i=0;i<3;i++) rtvecEigen(3, i) = 0.0;
    // rtvecEigen(3, 3) = 1.0;

    // Eigen::Matrix<double,4,1> point1_,point2_,point3_;
    // Eigen::Matrix<double,3,1> point1,point2,point3;
    // point1_=rtvecEigen*objpointsEigen[0];
    // point2_=rtvecEigen*objpointsEigen[2];
    // point3_=rtvecEigen*objpointsEigen[4];

    // point1=Eigen::Matrix<double,3,1>(point1_(0)/point1_(3),point1_(1)/point1_(3),point1_(2)/point1_(3));
    // point2=Eigen::Matrix<double,3,1>(point2_(0)/point2_(3),point2_(1)/point2_(3),point2_(2)/point2_(3));
    // point3=Eigen::Matrix<double,3,1>(point3_(0)/point3_(3),point3_(1)/point3_(3),point3_(2)/point3_(3));

    // PNPPlane=determinePlaneFromThreePoints(point1,point2,point3);

    // if(!PNPPlane.size()){
    //     RCLCPP_ERROR(this->get_logger(),"determinePlaneFromThreePoints fail!");
    //     return 0;
    // }

    // }

    // the num of points aroud corners of each plant 
    std::vector<int> scores;
    std::vector<double> distance;

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle([&CornerPoints](){
        Counter2f con;
        for(auto & i : CornerPoints){
            con.push_back(cv::Point2f(i.x,i.y));
        }
        return con;
    }(),center,radius);

    for(const auto & plant : ExtractedPlanes){
        if(IntersectionDistanceAlongZ(plant)<minPlaneDisThreshold) continue;
        int score=0;
        for(const auto i : *plant.points){
            Eigen::Matrix<double,4,1> cloudpointEigen;
            Eigen::Matrix<double,3,1> imagePoint;
            cloudpointEigen<<i.x, i.y, i.z, 1;
            imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
            if(std::abs(imagePoint(2))<1e-6) continue;
            imagePoint/=imagePoint(2);

            if(DistancePoints(cv::Point2f(imagePoint(0),imagePoint(1)),center)<=radius){
                score++;
            }
        }
        scores.push_back(score);
        distance.push_back(IntersectionDistanceAlongZ(plant));
    }

    Eigen::Vector4f coefficient;
    pcl::PointCloud<pcl::PointXYZ> PlanePointClouds;

    // 储存了2D点映射得到的3D点
    std::vector<cv::Point3d> Points3D;

    {

        std::vector<int> indexs;
        for(int i=0;i<ExtractedPlanes.size();i++){
            indexs.push_back(i);
        }

        std::sort(indexs.begin(),indexs.end(),[&scores](const int & a,const int & b){
            return scores[a]>scores[b];
        });

        for(int i=0;i<4;i++){
            coefficient[i]=ExtractedPlanes[indexs[0]].coefficients.values[i];
        }

        PlanePointClouds=*(ExtractedPlanes[indexs[0]].points);

    }
    
    bool ImagePointTo3DPoint_PlantCheck=ImagePointTo3DPoint_Plant(CornerPoints,coefficient,Points3D);
    if(ImagePointTo3DPoint_PlantCheck){
        RCLCPP_WARN(this->get_logger(),"ImagePointTo3DPoint_PlantCheck fail");
        return 0;
    }

    # ifdef test_pcl_manage
        
    {    
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg<pcl::PointXYZ>(PlanePointClouds,msg);
    msg.header.frame_id=ImageFrame;
    msg.header.stamp=this->now();
    pcl_test_point_cloud_pub->publish(msg);
    }

    for(const auto & e : CornerPoints){
        cv::circle(OriginalImage_pcl,cv::Point(e.x,e.y),CloseThresehold,cv::Scalar(225,225,225),2);
    }

    for(auto i : PlanePointClouds){
        Eigen::Matrix<double,4,1>  PlanePointCloudEigen;
        PlanePointCloudEigen<<i.x,i.y,i.z,1;
        Eigen::Matrix<double,3,1> I=cameraMatrixEigen*signMat*PlanePointCloudEigen;
        I/=I(2);
        cv::circle(OriginalImage_pcl,cv::Point(I(0),I(1)),1,cv::Scalar(130,100,22),-1);
        for(const auto & e : CornerPoints){
            if(DistancePoints(cv::Point2f(I(0),I(1)),e)<=CloseThresehold){
                cv::circle(OriginalImage_pcl,cv::Point(I(0),I(1)),1,cv::Scalar(225,225,225),-1);
                break;
            }
        }
    }

    #endif


    bool KabschAlgorithmCheck=KabschAlgorithm(objpoints,Points3D,tvec,rvec);
    if(KabschAlgorithmCheck){
        RCLCPP_WARN(this->get_logger(),"KabschAlgorithm fail");
        return KabschAlgorithmCheck;
    }
    return 1;

}

bool RedeemBox_detector::ImagePointTo3DPoint_Plant(const Counter2d& Points2D, const Eigen::VectorXf & plant, std::vector<cv::Point3d> &Points3D){
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

bool RedeemBox_detector::inCircle(const cv::Point2f & Center,
    const float & CornerPointsRadius,
    const Eigen::Matrix<double,3,1>& TestPoint){

    assert(CornerPointsRadius>0);

    float distance_2=((TestPoint(0)-Center.x)*(TestPoint(0)-Center.x)+
        (TestPoint(1)-Center.y)*(TestPoint(1)-Center.y));
    
    return distance_2<=CornerPointsRadius*CornerPointsRadius;
}


int RedeemBox_detector::MainPclManager(const cv::Mat& OriginalImage){

    // --- Start Timing for the whole function ---
    auto start_total = std::chrono::high_resolution_clock::now();

    // --- Start Timing for InTimeCloudUpdate ---
    auto start_update = std::chrono::high_resolution_clock::now();
    // InTimeCloudUpdate();
    auto end_update = std::chrono::high_resolution_clock::now();
    auto duration_update = std::chrono::duration_cast<std::chrono::milliseconds>(end_update - start_update).count();
    RCLCPP_INFO_STREAM(this->get_logger(), "[MainPclManager] InTimeCloudUpdate time: " << duration_update << " ms");

    // --- Start Timing for Image Copy ---
    auto start_copy = std::chrono::high_resolution_clock::now();
    OriginalImage.copyTo(OriginalImage_pcl);
    auto end_copy = std::chrono::high_resolution_clock::now();
    auto duration_copy = std::chrono::duration_cast<std::chrono::milliseconds>(end_copy - start_copy).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] Get frame and copy time: " << duration_copy << " ms"); // Combined get frame log here

    // --- Start Timing for PreProgress ---
    auto start_preproc = std::chrono::high_resolution_clock::now();
    cv::Mat BinaryImage=this->PreProgress(OriginalImage_pcl);
    auto end_preproc = std::chrono::high_resolution_clock::now();
    auto duration_preproc = std::chrono::duration_cast<std::chrono::milliseconds>(end_preproc - start_preproc).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] PreProgress finish. Time: " << duration_preproc << " ms");

    // --- Start Timing for TargetArrow ---
    auto start_targetarrow = std::chrono::high_resolution_clock::now();
    Counter2d CornerPoints=this->TargetArrow(BinaryImage,OriginalImage_pcl);
    auto end_targetarrow = std::chrono::high_resolution_clock::now();
    auto duration_targetarrow = std::chrono::duration_cast<std::chrono::milliseconds>(end_targetarrow - start_targetarrow).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] TargetArrow finish. Time: " << duration_targetarrow << " ms");

    // --- Start Timing for CornerPoints conversion ---
    auto start_conversion = std::chrono::high_resolution_clock::now();
    Counter2f CornerPointsf=[&CornerPoints](){
        Counter2f ans;
        for(auto & I : CornerPoints){
            ans.push_back(cv::Point2f(I.x,I.y));
        }
        return ans;
    }();
    auto end_conversion = std::chrono::high_resolution_clock::now();
    auto duration_conversion = std::chrono::duration_cast<std::chrono::milliseconds>(end_conversion - start_conversion).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] CornerPoints conversion time: " << duration_conversion << " ms");


    if(!CornerPoints.size()){
        RCLCPP_INFO(this->get_logger(),"target fail");
        // --- Log Total time even on failure ---
        auto end_total_fail = std::chrono::high_resolution_clock::now();
        auto duration_total_fail = std::chrono::duration_cast<std::chrono::milliseconds>(end_total_fail - start_total).count();
        RCLCPP_INFO_STREAM(this->get_logger(), "[MainPclManager] Total execution time (target fail): " << duration_total_fail << " ms");
        return 0;
    }

    // --- Start Timing for Bounding Box Calculation ---
    auto start_bbox = std::chrono::high_resolution_clock::now();
    //Rect bound counter
    cv::Rect boundingCounterBox=cv::boundingRect(CornerPointsf);
    //extend Rect
    boundingCounterBox.x=std::max(0,boundingCounterBox.x-boundingCounterBox.width/2);
    boundingCounterBox.y=std::max(0,boundingCounterBox.y-boundingCounterBox.height/2);
    boundingCounterBox.width=std::min(boundingCounterBox.width*2,OriginalImage_pcl.cols-boundingCounterBox.x);
    boundingCounterBox.height=std::min(boundingCounterBox.height*2,OriginalImage_pcl.rows-boundingCounterBox.y);
    auto end_bbox = std::chrono::high_resolution_clock::now();
    auto duration_bbox = std::chrono::duration_cast<std::chrono::milliseconds>(end_bbox - start_bbox).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] Bounding box calculation time: " << duration_bbox << " ms");

    // --- Start Timing for Min Enclosing Circle ---
    auto start_circle = std::chrono::high_resolution_clock::now();
    cv::Point2f center;float radius;
    cv::minEnclosingCircle(CornerPointsf,center,radius);
    radius*=2;
    auto end_circle = std::chrono::high_resolution_clock::now();
    auto duration_circle = std::chrono::duration_cast<std::chrono::milliseconds>(end_circle - start_circle).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"[MainPclManager] Min enclosing circle time: " << duration_circle << " ms");

    pcl::PointCloud<pcl::PointXYZ> PreprocessedCloudPoint;
    std::vector<Eigen::Matrix<double,3,1>> CloudPointImagePoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr VisibleCloudCameraFrame= std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    Cloudmtx.lock();

    VisibleCloudCameraFrame = removeHiddenPoints(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(InTimeCloud), ImageWidth, ImageHeight, cameraMatrixEigen(0,0), cameraMatrixEigen(1,1), cameraMatrixEigen(0,2), cameraMatrixEigen(1,2));
    
    Cloudmtx.unlock();

    # ifdef test_pcl_manage

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg<pcl::PointXYZ>(*VisibleCloudCameraFrame,msg);
    msg.header.frame_id=ImageFrame;
    msg.header.stamp=this->now();
    pcl_camera_point_cloud_pub->publish(msg);

    # endif

    // --- Start Timing for Point Cloud Filtering and Projection (including mutex) ---
    auto start_pcl_filter = std::chrono::high_resolution_clock::now();
    
    cv::Mat pnprvec,pnptvec;

    bool solvePnPcheck=cv::solvePnP(objpoints,CornerPoints,cameraMatrixMat,std::vector<double>{0,0,0,0,0},pnprvec,pnptvec,0,cv::SOLVEPNP_IPPE);

    if(!solvePnPcheck){
        RCLCPP_ERROR(this->get_logger(),"[GetTRvecPointCloud_PC] PnP failed!");
        return 0;
    }

    std::vector<double> PNPPlane;

    {// get plane

    cv::Mat rmat;
    cv::Rodrigues(pnprvec,rmat);
    Eigen::Matrix<double,4,4> rtvecEigen;

    for(int i=0;i<3;i++){
        for(int e=0;e<3;e++){
            rtvecEigen(i,e)=rmat.at<double>(i,e);
        }
        rtvecEigen(i,3)=pnptvec.at<double>(i);
    }
    for(int i=0;i<3;i++) rtvecEigen(3, i) = 0.0;
    rtvecEigen(3, 3) = 1.0;

    Eigen::Matrix<double,4,1> point1_,point2_,point3_;
    Eigen::Matrix<double,3,1> point1,point2,point3;
    point1_=rtvecEigen*objpointsEigen[0];
    point2_=rtvecEigen*objpointsEigen[2];
    point3_=rtvecEigen*objpointsEigen[4];

    point1=Eigen::Matrix<double,3,1>(point1_(0)/point1_(3),point1_(1)/point1_(3),point1_(2)/point1_(3));
    point2=Eigen::Matrix<double,3,1>(point2_(0)/point2_(3),point2_(1)/point2_(3),point2_(2)/point2_(3));
    point3=Eigen::Matrix<double,3,1>(point3_(0)/point3_(3),point3_(1)/point3_(3),point3_(2)/point3_(3));

    # ifdef test_pcl_manage

    # endif
    PNPPlane=determinePlaneFromThreePoints(point1,point2,point3);

    if(!PNPPlane.size()){
        RCLCPP_ERROR(this->get_logger(),"determinePlaneFromThreePoints fail!");
        return 0;
    }

    }

    {
        pcl::PointCloud<pcl::PointXYZ> PreprocessedCloudPointpnp;
        for(auto & i : *VisibleCloudCameraFrame){
            if(i.z>=1.5) continue;
            if(PointToPlaneDistance(i,PNPPlane)>0.03) continue;
            Eigen::Matrix<double,4,1> cloudpointEigen;
            Eigen::Matrix<double,3,1> imagePoint;
            cloudpointEigen<<i.x, i.y, i.z, 1;

            imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
            if(std::abs(imagePoint(2))<1e-6) continue;
            imagePoint/=imagePoint(2);

            // if(boundingCounterBox.x<=imagePoint(0)&&imagePoint(0)<=boundingCounterBox.x+boundingCounterBox.width&&
                // boundingCounterBox.y<=imagePoint(1)&&imagePoint(1)<=boundingCounterBox.y+boundingCounterBox.height
            if(inCircle(center,radius,imagePoint) // This 'if' condition needs to match the filtering logic
                ){
                    PreprocessedCloudPointpnp.push_back(i);
            }
        }

        auto FirstExtractedPlanes = segmentPlanesWithPoints(
                std::make_shared<pcl::PointCloud<pcl::PointXYZ> >(PreprocessedCloudPointpnp),
                ransacDistanceThreshold,
                ransacMaxIterations,
                PreprocessedCloudPointpnp.size()/2,
                1);

        if(!FirstExtractedPlanes.size()){
            RCLCPP_ERROR(this->get_logger(),"first segmentPlanesWithPoints failed");
            return 0;
        }

        for(auto & i : *VisibleCloudCameraFrame){
            if(i.z>=1.5) continue;
            if(PointToPlaneDistance(i,FirstExtractedPlanes[0].coefficients.values)>0.03) continue;
            Eigen::Matrix<double,4,1> cloudpointEigen;
            Eigen::Matrix<double,3,1> imagePoint;
            cloudpointEigen<<i.x, i.y, i.z, 1;

            imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
            if(std::abs(imagePoint(2))<1e-6) continue;
            imagePoint/=imagePoint(2);

            // if(boundingCounterBox.x<=imagePoint(0)&&imagePoint(0)<=boundingCounterBox.x+boundingCounterBox.width&&
                // boundingCounterBox.y<=imagePoint(1)&&imagePoint(1)<=boundingCounterBox.y+boundingCounterBox.height
            if(inCircle(center,radius,imagePoint) // This 'if' condition needs to match the filtering logic
                ){
                    PreprocessedCloudPoint.push_back(i);
            }
        }

    }


    auto end_pcl_filter = std::chrono::high_resolution_clock::now();
    auto duration_pcl_filter = std::chrono::duration_cast<std::chrono::milliseconds>(end_pcl_filter - start_pcl_filter).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"Select the appropriate point cloud with size :"<<PreprocessedCloudPoint.size() << ". Time: " << duration_pcl_filter << " ms");


    # ifdef test_pcl_manage
    // --- Start Timing for Drawing (Conditional) ---
    auto start_draw = std::chrono::high_resolution_clock::now();

    DrawPnPResult(OriginalImage_pcl,pnprvec,pnptvec,cv::Scalar(33,223,123),1,cv::Point(-312312,-31231));

    for(auto i : CloudPointImagePoint){
        cv::circle(OriginalImage_pcl,cv::Point(i(0),i(1)),1,cv::Scalar(22,33,130),-1);
    }
    DrawRect(OriginalImage_pcl,boundingCounterBox,cv::Scalar(102,32,210),1);
    cv::circle(OriginalImage_pcl,cv::Point(center.x,center.y),radius,cv::Scalar(102,32,210));
    auto end_draw = std::chrono::high_resolution_clock::now();
    auto duration_draw = std::chrono::duration_cast<std::chrono::milliseconds>(end_draw - start_draw).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"Drawing time: " << duration_draw << " ms");
    RCLCPP_INFO(this->get_logger(),"DrawRect and circle finish!"); // Keep original log if desired
    # endif

    cv::Mat tvec,rvec;
    // --- Start Timing for GetTRvecPointCloud_PC (PnP) ---
    auto start_pnp = std::chrono::high_resolution_clock::now();
    bool GetTRvecPointCloud_PCcheck=GetTRvecPointCloud_PC(PreprocessedCloudPoint,CornerPoints,tvec,rvec);
    auto end_pnp = std::chrono::high_resolution_clock::now();
    auto duration_pnp = std::chrono::duration_cast<std::chrono::milliseconds>(end_pnp - start_pnp).count();

    if(!GetTRvecPointCloud_PCcheck){
        RCLCPP_ERROR_STREAM(this->get_logger(),"MainPclManager GetTRvecPointCloud_PC fail! Time taken: " << duration_pnp << " ms");
        // --- Log Total time even on PnP failure ---
        auto end_total_fail_pnp = std::chrono::high_resolution_clock::now();
        auto duration_total_fail_pnp = std::chrono::duration_cast<std::chrono::milliseconds>(end_total_fail_pnp - start_total).count();
        RCLCPP_INFO_STREAM(this->get_logger(), "[MainPclManager] Total execution time (PnP fail): " << duration_total_fail_pnp << " ms");
        return 0;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"GetTRvecPointCloud_PC time: " << duration_pnp << " ms");

    // --- Start Timing for SendBoxPosition ---
    auto start_send = std::chrono::high_resolution_clock::now();
    SendBoxPosition(tvec,rvec);
    auto end_send = std::chrono::high_resolution_clock::now();
    auto duration_send = std::chrono::duration_cast<std::chrono::milliseconds>(end_send - start_send).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"SendBoxPosition time: " << duration_send << " ms");

    #ifdef SyncPubBoxPos
    // --- Start Timing for SyncPubBoxPos (Conditional) ---
    auto start_sync = std::chrono::high_resolution_clock::now();
    cloudressMtx.lock();
    cloudress.push(PnPresult(tvec,rvec,this->now()));
    cloudressMtx.unlock();
    auto end_sync = std::chrono::high_resolution_clock::now();
    auto duration_sync = std::chrono::duration_cast<std::chrono::milliseconds>(end_sync - start_sync).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"SyncPubBoxPos time: " << duration_sync << " ms");
    #endif

    # ifdef test_pcl_manage
    // --- Start Timing for Final Drawing/Display (Conditional) ---
    auto start_final_draw = std::chrono::high_resolution_clock::now();
    DrawPnPResult(OriginalImage_pcl,rvec,tvec,cv::Scalar(225,80,22),3,cv::Point(20,40));

    std::stringstream test_pcl_managesss;
    test_pcl_managesss<<rvec<<"\n"<<tvec;
    RCLCPP_INFO(this->get_logger(),"test_pcl_managesss : %s",test_pcl_managesss.str().c_str());
    cv::imshow("OriginalImage_pcl",OriginalImage_pcl);
    cv::waitKey(1);
    auto end_final_draw = std::chrono::high_resolution_clock::now();
    auto duration_final_draw = std::chrono::duration_cast<std::chrono::milliseconds>(end_final_draw - start_final_draw).count();
    RCLCPP_INFO_STREAM(this->get_logger(),"Final draw/display time: " << duration_final_draw << " ms");
    # endif

    // --- End Timing for the whole function ---
    auto end_total = std::chrono::high_resolution_clock::now();
    auto duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total).count();
    RCLCPP_INFO_STREAM(this->get_logger(), "[MainPclManager] Total execution time: " << duration_total << " ms");

    return 1;
}


std::vector<PlaneData> segmentPlanesWithPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
    double distanceThreshold,
    int MaxIterations,
    int minInliersNum,
    int maxPlaneNum,
    const rclcpp::Logger& logger){
    
    std::vector<PlaneData> segmented_planes;

    if (!original_cloud || original_cloud->empty()) {
        RCLCPP_ERROR(logger,"Input cloud is empty or invalid.");
        return segmented_planes;
    }

    std::vector<int> current_indices(original_cloud->size());
    std::iota(current_indices.begin(), current_indices.end(), 0);
 
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(MaxIterations);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    RCLCPP_INFO_STREAM(logger,"Starting plane segmentation with " << original_cloud->size() << " points.");

    while (current_indices.size() > minInliersNum&& segmented_planes.size()<maxPlaneNum){
        // Create a temporary cloud from the points referenced by current_indices in the original cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndices::Ptr temp_indices_ptr(new pcl::PointIndices()); // Indices used to extract temp_cloud
        temp_indices_ptr->indices = current_indices; // Use the current_indices to extract points

        // Use ExtractIndices to actually create temp_cloud from original_cloud based on current_indices
        extract.setInputCloud(original_cloud);
        extract.setIndices(temp_indices_ptr);
        extract.setNegative(false); // Extract the points specified by temp_indices_ptr
        extract.filter(*temp_cloud);

        if (temp_cloud->size() != current_indices.size()) {
            RCLCPP_ERROR_STREAM(logger,"Error: temp_cloud size mismatch (" << temp_cloud->size() << ") with current_indices size (" << current_indices.size() << ")!");
            break; // Should not happen in normal circumstances
        }

        if (temp_cloud->empty()) {
            RCLCPP_INFO(logger,"Temporary cloud is empty. Stopping segmentation.");
            break;
        }


        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_temp(new pcl::PointIndices); // Indices relative to temp_cloud

        // Perform segmentation on the temporary cloud
        seg.setInputCloud(temp_cloud);
        seg.segment(*inliers_temp, *coefficients);

        if (inliers_temp->indices.empty() || inliers_temp->indices.size() < minInliersNum){
            // Could not find a plane with enough inliers in the current subset
            RCLCPP_INFO_STREAM(logger,"Could not find a plane with enough inliers (" << inliers_temp->indices.size() << ") in the current cloud subset (" << temp_cloud->size() << " points). Stopping." );
            break;
        }

        // --- Found a plane ---
        PlaneData current_plane_data;
        current_plane_data.coefficients = *coefficients;

        // Get the original indices corresponding to the inliers found in temp_cloud
        pcl::PointIndices::Ptr inliers_original(new pcl::PointIndices());
        inliers_original->indices.reserve(inliers_temp->indices.size());
        for (int index_in_temp : inliers_temp->indices) {
            if (index_in_temp < current_indices.size()) { // Safeguard
                inliers_original->indices.push_back(current_indices[index_in_temp]);
            } 
            else {
                RCLCPP_ERROR_STREAM(logger,"Error: Index in inliers_temp (" << index_in_temp<< ") out of bounds for current_indices (" << current_indices.size() << ")!");
            }
        }

        // Extract the actual points of the plane from the original cloud using original indices
        current_plane_data.points.reset(new pcl::PointCloud<pcl::PointXYZ>()); // Allocate memory for the plane points
        pcl::copyPointCloud(*original_cloud, *inliers_original, *current_plane_data.points);

        // Add the plane data to the result vector
        segmented_planes.push_back(current_plane_data);

        // --- Prepare for the next iteration: Find the remaining points (outliers) ---

        // We need the indices of the outliers in temp_cloud, and then map them back to original indices.
        // Indices of outliers in temp_cloud = all indices in temp_cloud - indices of inliers in temp_cloud

        std::vector<int> all_temp_indices(temp_cloud->size());
        std::iota(all_temp_indices.begin(), all_temp_indices.end(), 0);
        std::vector<int> inliers_vec = inliers_temp->indices;
        std::sort(inliers_vec.begin(), inliers_vec.end()); // std::set_difference requires sorted ranges

        std::vector<int> outliers_temp_indices_vec;
        std::set_difference(
            all_temp_indices.begin(), all_temp_indices.end(),
            inliers_vec.begin(), inliers_vec.end(),
            std::back_inserter(outliers_temp_indices_vec));

        // Update current_indices to contain only the original indices of the outliers
        std::vector<int> next_current_indices;
        next_current_indices.reserve(outliers_temp_indices_vec.size());
        for (int index_in_temp_outlier : outliers_temp_indices_vec) {
             if (index_in_temp_outlier < current_indices.size()) { // Safeguard
                next_current_indices.push_back(current_indices[index_in_temp_outlier]);
             } else {
                  RCLCPP_ERROR_STREAM(logger, "Error: Index in outliers_temp_indices_vec (" << index_in_temp_outlier << ") out of bounds for current_indices (" << current_indices.size() << ")!" );
             }
        }
        current_indices = next_current_indices;

        // Optional: print progress
        RCLCPP_INFO_STREAM(logger,"Found plane " << segmented_planes.size() << " with " << inliers_temp->indices.size() << " inliers. " << current_indices.size() << " points remaining for next iteration.");
    }
    

    RCLCPP_INFO_STREAM(logger,"Segmentation finished. Found " << segmented_planes.size() << " planes.");

    return segmented_planes;

}
/*
接下来的思路：

我们发现以确定的坐标轴和原点得到准确坐标的路径有两个

1. 先拟合平面，然后再根据拟合出来的平面，算出得到的角点的空间位置
2. 直接通过选择的角点，得到的直线，从已知的点云中选择点来组成坐标系
3. 直接边缘拟合，得到边框的确切边缘，然后用数学关系直接求得兑换框中心点位置
*/

}
