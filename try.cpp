#include "target_redeem_box/RedeemBox_detector.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
#include <pcl/common/common.h>
#include <numeric> // for std::iota
#include <pcl/copy_point_cloud.h> // for pcl::copyPointCloud
#include <algorithm> // for std::set_difference, std::sort
#include <limits> // for std::numeric_limits
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp> // For cv::eigen2cv

namespace Engineering_robot_RM2025_Pnx{

// Assuming RoiPointInfo and PlaneInfoLocalIndices are defined in the header

// Assuming segmentMultiplePlanesLocal is defined as a member function as shown above


bool RedeemBox_detector::GetTRvecPointCloud_PC(
    const pcl::PointCloud<pcl::PointXYZ> & inputPointCloudWiderROI, // Input ROI cloud by const reference
    Counter2d CornerPoints,                                        // Target 2D corner points
    cv::Mat & tvec,                                               // Output: translation vector
    cv::Mat & rvec                                                // Output: rotation vector
)
{
    // Create a shared pointer from the const reference for PCL operations
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr =
        std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(inputPointCloudWiderROI);


    if (input_cloud_ptr->empty()) {
        RCLCPP_WARN(this->get_logger(), "Input ROI point cloud is empty");
        return false; // Indicate failure
    }
    if (CornerPoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "No corner points provided");
        return false; // Indicate failure
    }
     if (CornerPoints.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Not enough corner points provided (%zu) for PnP.", CornerPoints.size());
        return false; // Need at least 4 points for PnP
    }


    // --- Step 0: Compute 3D-to-2D mapping for ROI points locally ---
    std::vector<RoiPointInfo> roi_points_info;
    roi_points_info.reserve(input_cloud_ptr->size()); // Reserve space

    for (size_t i = 0; i < input_cloud_ptr->size(); ++i) {
        const auto& point3D = input_cloud_ptr->points[i];

        Eigen::Matrix<double,4,1> cloudpointEigen;
        cloudpointEigen << point3D.x, point3D.y, point3D.z, 1;

        // Assuming cameraMatrixEigen and signMat are accessible member variables (Eigen::Matrix<double, 3, 3/4>)
        // You might need to adjust the matrix multiplication depending on signMat dimensions
        Eigen::Matrix<double,3,1> imagePoint_homogeneous = cameraMatrixEigen * signMat.block<3,4>(0,0) * cloudpointEigen; // Assuming signMat is 3x4 or can be block-extracted

        // Check if point is behind the camera or projection is invalid
        if (std::abs(imagePoint_homogeneous(2)) < 1e-6) continue;

        // Normalize to get pixel coordinates
        cv::Point2d imagePoint(imagePoint_homogeneous(0) / imagePoint_homogeneous(2),
                               imagePoint_homogeneous(1) / imagePoint_homogeneous(2));

        RoiPointInfo rp_info;
        rp_info.roi_cloud_index = i; // Index in the input_cloud_ptr
        rp_info.point2D_proj = imagePoint;
        roi_points_info.push_back(rp_info);
    }

     if (roi_points_info.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid 3D points in ROI could be projected to 2D.");
        return false;
    }


    // --- Step 1: Segment multiple planes from the ROI cloud ---
    // Use class parameters for segmentation thresholds
    double segment_distance_thresh = ransacDistanceThreshold;
    int segment_min_inliers = 20; // Define a suitable minimum for candidate planes
                                 // You might want to make this configurable (e.g., minCandidatePlaneInliers)

    // Call the helper function to segment planes within the input ROI cloud
    // Indices in candidate_planes are relative to input_cloud_ptr
    std::vector<PlaneInfoLocalIndices> candidate_planes = segmentMultiplePlanesLocal(
        input_cloud_ptr, segment_distance_thresh, segment_min_inliers);

    if (candidate_planes.empty()) {
        RCLCPP_WARN(this->get_logger(), "No candidate planes found in ROI cloud.");
        return false; // No planes segmented
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu candidate planes.", candidate_planes.size());

    // --- Step 2: Evaluate and select the best plane based on CornerPoints ---

    int best_plane_index = -1;
    int max_score = -1;

    // For efficient lookup of plane inliers in the input_cloud_ptr
    std::vector<std::vector<bool>> plane_inlier_masks(candidate_planes.size());
    for(size_t i = 0; i < candidate_planes.size(); ++i) {
        plane_inlier_masks[i].resize(input_cloud_ptr->size(), false);
        if (candidate_planes[i].inlier_indices) {
            for (int idx : candidate_planes[i].inlier_indices->indices) {
                if (idx >= 0 && idx < input_cloud_ptr->size()) { // Safeguard
                    plane_inlier_masks[i][idx] = true;
                } else {
                     RCLCPP_ERROR(this->get_logger(), "Invalid inlier index %d in plane %zu (Input cloud size %zu).",
                                  idx, i, input_cloud_ptr->size());
                }
            }
        }
    }

    double corner_point_match_threshold_sq = 5.0 * 5.0; // Max squared 2D distance for a projected point to "match" a CornerPoint for scoring

    for (size_t i = 0; i < candidate_planes.size(); ++i) {
        int current_score = 0;

        // Evaluate how well CornerPoints align with this plane
        // For each CornerPoint, find if there is a close point in roi_points_info
        // whose 3D origin is an inlier of the current plane.

        for (const auto& corner_pt_2d : CornerPoints) {
            double min_dist_sq = std::numeric_limits<double>::max();
            int closest_roi_info_index = -1; // Index in roi_points_info vector

            // Find the point in roi_points_info whose 2D projection is closest to the current CornerPoint
            for (size_t j = 0; j < roi_points_info.size(); ++j) {
                 // Check if the 2D projection is within the bounds of the image (optional but good practice)
                // if (roi_points_info[j].point2D_proj.x >= 0 && roi_points_info[j].point2D_proj.x < your_image_width &&
                //     roi_points_info[j].point2D_proj.y >= 0 && roi_points_info[j].point2D_proj.y < your_image_height)
                {
                    double dist_sq = (roi_points_info[j].point2D_proj.x - corner_pt_2d.x) * (roi_points_info[j].point2D_proj.x - corner_pt_2d.x) +
                                   (roi_points_info[j].point2D_proj.y - corner_pt_2d.y) * (roi_points_info[j].point2D_proj.y - corner_pt_2d.y);
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        closest_roi_info_index = j; // Index in roi_points_info
                    }
                }
            }

            if (closest_roi_info_index != -1 && min_dist_sq < corner_point_match_threshold_sq) {
                // We found a close point in roi_points_info based on 2D projection.
                // Now check if its corresponding 3D point (at index roi_points_info[closest_roi_info_index].roi_cloud_index
                // in input_cloud_ptr) is an inlier of the current plane.
                 int roi_idx = roi_points_info[closest_roi_info_index].roi_cloud_index;
                 if (roi_idx >= 0 && roi_idx < plane_inlier_masks[i].size() && plane_inlier_masks[i][roi_idx]) {
                     current_score++; // This plane supports this CornerPoint
                 }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Candidate Plane %zu score: %d (out of %zu CornerPoints)", i, current_score, CornerPoints.size());

        // Select the plane with the most supported CornerPoints
        if (current_score > max_score) {
            max_score = current_score;
            best_plane_index = i;
        }
    }

    // Define a minimum score threshold to consider a plane valid for pose estimation
    // Require at least 4 points, maybe a percentage of corners
    int min_required_supported_points = std::max(4, (int)(CornerPoints.size() * 0.75));

    if (best_plane_index == -1 || max_score < min_required_supported_points) {
         RCLCPP_WARN(this->get_logger(), "Could not confidently determine the best plane for CornerPoints. Max score: %d/%zu. Required: %d",
                    max_score, CornerPoints.size(), min_required_supported_points);
         return false;
    }

    RCLCPP_INFO(this->get_logger(), "Selected plane %d as the best plane with score %d.", best_plane_index, max_score);

    // --- Step 3: Get 3D points corresponding to CornerPoints using the best plane ---

    pcl::ModelCoefficients& best_plane_coefficients = candidate_planes[best_plane_index].coefficients;
    std::vector<cv::Point3d> CornerPoints3D;
    CornerPoints3D.reserve(CornerPoints.size());

    double final_corner_match_threshold_sq = 10.0 * 10.0; // Max squared 2D distance for a direct point match from the best plane

    // Now, for each CornerPoint, find the closest point in roi_points_info
    // whose 3D point is an inlier of the *best* plane and whose 2D projection is close enough.
    // This point's 3D coordinate will be used for PnP.
    for (const auto& corner_pt_2d : CornerPoints) {
         double min_dist_sq = std::numeric_limits<double>::max();
         bool found_direct_match = false;
         cv::Point3d best_match_3d_direct;

         for (const auto& rp_info : roi_points_info) {
             int roi_idx = rp_info.roi_cloud_index;
             // Check if this ROI point belongs to the best plane
             if (roi_idx >= 0 && roi_idx < plane_inlier_masks[best_plane_index].size() &&
                 plane_inlier_masks[best_plane_index][roi_idx])
             {
                 // This point is on the best plane, check its 2D projection distance to the CornerPoint
                 double dist_sq = (rp_info.point2D_proj.x - corner_pt_2d.x) * (rp_info.point2D_proj.x - corner_pt_2d.x) +
                                (rp_info.point2D_proj.y - corner_pt_2d.y) * (rp_info.point2D_proj.y - corner_pt_2d.y);

                 if (dist_sq < min_dist_sq) {
                     min_dist_sq = dist_sq;
                     // Get the 3D point from the original input cloud using the stored index
                     const auto& point3D = input_cloud_ptr->points[roi_idx];
                     best_match_3d_direct = cv::Point3d(point3D.x, point3D.y, point3D.z);
                     found_direct_match = true; // Found at least one point on the plane
                 }
             }
         }

         // Use the best direct match if it's within the threshold
         if (found_direct_match && min_dist_sq < final_corner_match_threshold_sq) {
              CornerPoints3D.push_back(best_match_3d_direct);
              // RCLCPP_INFO(this->get_logger(), "Using direct 3D point match for CornerPoint (2D: %f, %f). Dist sq: %f", corner_pt_2d.x, corner_pt_2d.y, min_dist_sq);
         } else {
             // Fallback: Project the 2D CornerPoint onto the fitted best plane
             RCLCPP_WARN(this->get_logger(), "No good direct 3D point match found on the best plane for CornerPoint (2D: %f, %f). Closest dist sq: %f (Threshold: %f). Falling back to projection.",
                        corner_pt_2d.x, corner_pt_2d.y, min_dist_sq, final_corner_match_threshold_sq);

             Eigen::Matrix<double,3,1> point2dlin, point3dnoZlin;
             point2dlin << corner_pt_2d.x, corner_pt_2d.y, 1.0; // Make it homogeneous 2D

             // Project into camera coordinates at depth 1
             // point3dnoZlin = InverseCameraMatrixEigen * point2dlin; // Assuming InverseCameraMatrixEigen is 3x3
             // If cameraMatrixEigen is 3x3, InverseCameraMatrixEigen should also be 3x3.
             // Need to get camera matrix as Eigen 3x3 if it's not already.
             // Assuming cameraMatrixEigen is cv::Mat, convert it first.
             cv::Mat cameraMatrix_cv;
             // Assuming cameraMatrixEigen is an Eigen matrix, convert to cv::Mat
             // eigen2cv(cameraMatrixEigen, cameraMatrix_cv); // Needs <opencv2/core/eigen.hpp>
             // cv::Mat inverseCameraMatrix_cv = cameraMatrix_cv.inv();
             // Eigen::Matrix3d inverseCameraMatrixEigen;
             // cv2eigen(inverseCameraMatrix_cv, inverseCameraMatrixEigen);

             // Let's assume InverseCameraMatrixEigen is an Eigen::Matrix3d member variable
             point3dnoZlin = InverseCameraMatrixEigen * point2dlin;

             // The ray direction from camera origin is [point3dnoZlin(0), point3dnoZlin(1), point3dnoZlin(2)]
             // Any point on the ray is t * [point3dnoZlin(0), point3dnoZlin(1), point3dnoZlin(2)]^T
             // Note: If InverseCameraMatrixEigen was used to get points at Z=1, point3dnoZlin(2) would be 1.0.
             // Depending on how InverseCameraMatrixEigen is defined, the vector point3dnoZlin represents the direction.
             // Let's assume point3dnoZlin already represents a direction vector from the camera center.

             Eigen::Vector4f plane_coeffs_eigen(best_plane_coefficients.values[0],
                                               best_plane_coefficients.values[1],
                                               best_plane_coefficients.values[2],
                                               best_plane_coefficients.values[3]); // Ax + By + Cz + D = 0

             // Intersection of ray t*point3dnoZlin and plane:
             // A*(t*point3dnoZlin(0)) + B*(t*point3dnoZlin(1)) + C*(t*point3dnoZlin(2)) + D = 0
             // t * (A*point3dnoZlin(0) + B*point3dnoZlin(1) + C*point3dnoZlin(2)) = -D
             // t = -D / (A*point3dnoZlin(0) + B*point3dnoZlin(1) + C*point3dnoZlin(2))

             double denominator = plane_coeffs_eigen(0) * point3dnoZlin(0) +
                                  plane_coeffs_eigen(1) * point3dnoZlin(1) +
                                  plane_coeffs_eigen(2) * point3dnoZlin(2);

             if (std::abs(denominator) < 1e-9) { // Check if ray is parallel to plane
                 RCLCPP_ERROR(this->get_logger(), "Projection ray is parallel to the best plane for CornerPoint (2D: %f, %f). Denominator: %f",
                              corner_pt_2d.x, corner_pt_2d.y, denominator);
                 CornerPoints3D.clear(); // Clear any points already added
                 return false; // Cannot project, PnP will likely fail
             }

             double t = -plane_coeffs_eigen(3) / denominator; // -D / (A*dx + B*dy + C*dz) where [dx, dy, dz] is ray direction
             cv::Point3d projected_3d(point3dnoZlin(0) * t, point3dnoZlin(1) * t, point3dnoZlin(2) * t);
             CornerPoints3D.push_back(projected_3d);
             RCLCPP_INFO(this->get_logger(), "Projected 2D point onto best plane: (%f, %f, %f)", projected_3d.x, projected_3d.y, projected_3d.z);
         }
    }

     // Ensure we found the expected number of 3D points for PnP
    if (CornerPoints3D.size() != CornerPoints.size()) {
         RCLCPP_ERROR(this->get_logger(), "Mismatch between CornerPoints size (%zu) and found 3D points size (%zu) for PnP after processing all points.",
                    CornerPoints.size(), CornerPoints3D.size());
         return false; // PnP will likely fail if point counts don't match
    }
    // Also check if any points were actually added (e.g., if CornerPoints was not empty but projection failed for all)
    if (CornerPoints3D.empty()){
        RCLCPP_ERROR(this->get_logger(), "No 3D points were successfully determined for CornerPoints.");
        return false;
    }


    // --- Step 4: PnP / Kabsch Algorithm ---
    // Use the found CornerPoints3D (observed 3D points) and your predefined objpoints (model 3D points)
    // Assuming KabschAlgorithm is a member function of RedeemBox_detector
    // and objpoints is a member variable (std::vector<cv::Point3d>)

    bool KabschAlgorithmCheck = KabschAlgorithm(this->objpoints, CornerPoints3D, tvec, rvec);
    if(KabschAlgorithmCheck){
        RCLCPP_WARN(this->get_logger(),"KabschAlgorithm fail after best plane selection.");
        return false; // Indicate failure
    }
    RCLCPP_INFO(this->get_logger(),"KabschAlgorithm OK!");

    # ifdef test_pcl_manage
     // Optional: Publish the best plane cloud for visualization
     // The inlier indices of the best plane are in candidate_planes[best_plane_index].inlier_indices,
     // which are relative to the input_cloud_ptr.
     pcl::PointCloud<pcl::PointXYZ>::Ptr best_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::ExtractIndices<pcl::PointXYZ> extract_best_plane;
     extract_best_plane.setInputCloud(input_cloud_ptr);
     extract_best_plane.setIndices(candidate_planes[best_plane_index].inlier_indices);
     extract_best_plane.setNegative(false);
     extract_best_plane.filter(*best_plane_cloud);

     sensor_msgs::msg::PointCloud2 best_plane_msg;
     pcl::toROSMsg<pcl::PointXYZ>(*best_plane_cloud, best_plane_msg);
     best_plane_msg.header.frame_id = ImageFrame; // Use appropriate frame_id from your class
     best_plane_msg.header.stamp = this->now(); // Use current time
     this->pcl_test_point_cloud_pub->publish(best_plane_msg); // Assuming this publisher exists
     RCLCPP_INFO(this->get_logger(), "Published best plane cloud with %zu points.", best_plane_cloud->size());

    #endif


    return true; // Indicate success
}

// ... other member functions ...

} // namespace Engineering_robot_RM2025_Pnx