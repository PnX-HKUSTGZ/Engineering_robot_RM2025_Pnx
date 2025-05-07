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

    # endif

    ransacDistanceThreshold=PCLManagerConfig["ransacDistanceThreshold"].as<double>();
    ransacMaxIterations=PCLManagerConfig["ransacMaxIterations"].as<int>();
    ransacMinInliersNum=PCLManagerConfig["ransacMinInliersNum"].as<int>();
    CloseThresehold=PCLManagerConfig["CloseThresehold"].as<double>();

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
        ransacMinInliersNum);

    if(!ExtractedPlanes.size()){
        RCLCPP_ERROR(this->get_logger(),"segmentPlanesWithPoints fail to get any plane!");
        return 0;
    }
    
    // choose the best plant

    // the num of points aroud corners of each plant 
    std::vector<int> scores;

    for(const auto & plant : ExtractedPlanes){
        int score=0;
        for(const auto i : *plant.points){
            Eigen::Matrix<double,4,1> cloudpointEigen;
            Eigen::Matrix<double,3,1> imagePoint;
            cloudpointEigen<<i.x, i.y, i.z, 1;
            imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
            if(std::abs(imagePoint(2))<1e-6) continue;
            imagePoint/=imagePoint(2);

            for(const auto & i : CornerPoints){
                if(DistancePoints(cv::Point2f(imagePoint(0),imagePoint(1)),i)<=CloseThresehold){
                    score++;
                }
            }
        }
        scores.push_back(score);
    }

    Eigen::Vector4f coefficient;
    pcl::PointCloud<pcl::PointXYZ> PlanePointClouds;

    // 储存了2D点映射得到的3D点
    std::vector<cv::Point3d> Points3D;

    {
        int maxIndex=-1;
        int maxscore=0;
        for(int i=0;i<scores.size();i++){
            if(maxscore<scores[i]){
                maxscore=scores[i];
                maxIndex=i;
            }
        }

        if(maxIndex==-1){
            RCLCPP_ERROR(this->get_logger(),"fail to find a plane close to the corners");
            return 0;
        }

        for(int i=0;i<4;i++){
            coefficient[i]=ExtractedPlanes[maxIndex].coefficients.values[i];
        }

        PlanePointClouds=*(ExtractedPlanes[maxIndex].points);

    }
    
    bool ImagePointTo3DPoint_PlantCheck=ImagePointTo3DPoint_Plant(CornerPoints,coefficient,Points3D);
    if(ImagePointTo3DPoint_PlantCheck){
        RCLCPP_WARN(this->get_logger(),"ImagePointTo3DPoint_PlantCheck fail");
        return 0;
    }

    # ifdef test_pcl_manage
        
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg<pcl::PointXYZ>(PlanePointClouds,msg);
    msg.header.frame_id=ImageFrame;
    msg.header.stamp=this->now();
    pcl_test_point_cloud_pub->publish(msg);

    for(auto i : PlanePointClouds){
        Eigen::Matrix<double,4,1>  PlanePointCloudEigen;
        PlanePointCloudEigen<<i.x,i.y,i.z,1;
        Eigen::Matrix<double,3,1> I=cameraMatrixEigen*signMat*PlanePointCloudEigen;
        I/=I(2);
        cv::circle(OriginalImage_pcl,cv::Point(I(0),I(1)),1,cv::Scalar(130,100,22),-1);
        for(const auto & e : CornerPoints){
            if(DistancePoints(cv::Point2f(I(0),I(1)),e)<=CloseThresehold){
                cv::circle(OriginalImage_pcl,cv::Point(I(0),I(1)),1,cv::Scalar(30,23,122),-1);
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
    InTimeCloudUpdate();
    OriginalImage.copyTo(OriginalImage_pcl);
    RCLCPP_INFO(this->get_logger(), "Get frame");

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
        return 0;
    }

    //Rect bound counter
    cv::Rect boundingCounterBox=cv::boundingRect(CornerPointsf);

    //extend Rect
    // boundingCounterBox.x=std::max(0,boundingCounterBox.x-boundingCounterBox.width/2);
    // boundingCounterBox.y=std::max(0,boundingCounterBox.y-boundingCounterBox.height/2);
    // boundingCounterBox.width=std::min(boundingCounterBox.width*2,OriginalImage_pcl.cols-boundingCounterBox.x);
    // boundingCounterBox.height=std::min(boundingCounterBox.height*2,OriginalImage_pcl.rows-boundingCounterBox.y);

    pcl::PointCloud<pcl::PointXYZ> PreprocessedCloudPoint;
    std::vector<Eigen::Matrix<double,3,1>> CloudPointImagePoint;

    Cloudmtx.lock();
    
    for(auto & i : InTimeCloud){
        if(i.z>=1.2) continue;
        Eigen::Matrix<double,4,1> cloudpointEigen;
        Eigen::Matrix<double,3,1> imagePoint;
        cloudpointEigen<<i.x, i.y, i.z, 1;

        imagePoint=cameraMatrixEigen*signMat*cloudpointEigen;
        if(std::abs(imagePoint(2))<1e-6) continue;
        imagePoint/=imagePoint(2);

        if(boundingCounterBox.x<=imagePoint(0)&&imagePoint(0)<=boundingCounterBox.x+boundingCounterBox.width&&
            boundingCounterBox.y<=imagePoint(1)&&imagePoint(1)<=boundingCounterBox.y+boundingCounterBox.height
            ){
                PreprocessedCloudPoint.push_back(i);
                CloudPointImagePoint.push_back(std::move(imagePoint));
        }
    }

    Cloudmtx.unlock();
    

    # ifdef test_pcl_manage

    for(auto i : CloudPointImagePoint){
        cv::circle(OriginalImage_pcl,cv::Point(i(0),i(1)),1,cv::Scalar(22,33,130),-1);
    }

    DrawRect(OriginalImage_pcl,boundingCounterBox,cv::Scalar(102,32,210),1);
    

    # endif

    cv::Mat tvec,rvec;
    bool GetTRvecPointCloud_PCcheck=GetTRvecPointCloud_PC(PreprocessedCloudPoint,CornerPoints,tvec,rvec);

    if(!GetTRvecPointCloud_PCcheck){
        RCLCPP_ERROR(this->get_logger(),"MainPclManager GetTRvecPointCloud_PC fail!");
        return 0;
    }

    SendBoxPosition(tvec,rvec);

    #ifdef SyncPubBoxPos
    cloudressMtx.lock();
    cloudress.push(PnPresult(tvec,rvec,this->now()));
    cloudressMtx.unlock();
    #endif

    # ifdef test_pcl_manage
    DrawPnPResult(OriginalImage_pcl,rvec,tvec,cv::Scalar(225,80,22),3,cv::Point(20,40));

    std::stringstream test_pcl_managesss;
    test_pcl_managesss<<rvec<<"\n"<<tvec;
    RCLCPP_INFO(this->get_logger(),"test_pcl_managesss : %s",test_pcl_managesss.str().c_str());
    cv::imshow("OriginalImage_pcl",OriginalImage_pcl);
    cv::waitKey(1);
    # endif

    return 1;
}


std::vector<PlaneData> segmentPlanesWithPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
    double distanceThreshold,
    int MaxIterations,
    int minInliersNum,
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

    while (current_indices.size() > minInliersNum){
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
