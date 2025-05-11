#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp> // For cv::Rodrigues
#include <opencv2/imgproc.hpp> // For cv::hconcat (usually here, or in core)
#include <vector>
#include <iostream>

// 假设 rvec 和 tvec 已经根据您的应用场景计算或加载
// 例如：
// cv::Mat rvec = (cv::Mat_<double>(3,1) << 0.1, 0.2, 0.3);
// cv::Mat tvec = (cv::Mat_<double>(3,1) << 0.05, -0.1, 0.5);

// 这里使用占位符 rvec 和 tvec，请替换为您实际的值
cv::Mat rvec; // 假设这是您的旋转向量 (3x1 或 1x3, CV_32F 或 CV_64F)
cv::Mat tvec; // 假设这是您的平移向量 (3x1 或 1x3, CV_32F 或 CV_64F)

// --- 请在这里填充实际的 rvec 和 tvec ---
// 示例填充 (请根据您的实际情况修改)
rvec = (cv::Mat_<double>(3,1) << 0.1, 0.2, 0.3);
tvec = (cv::Mat_<double>(3,1) << 0.05, -0.1, 0.5);
// ---------------------------------------

// 您的世界坐标系下的点
std::vector<cv::Point3d> points_world_vec = {
    cv::Point3d(0.144, 0, 0.0455),
    cv::Point3d(0.144, -0.1, 0.1455),
    cv::Point3d(0.144, 0.1, 0.1455)
};

// 1. 将 std::vector<cv::Point3d> 转换为 cv::Mat
// cv::transform 期望输入是一个 Nx3 或 1xN 的多通道矩阵 (这里 N 是点数)
// 对于 Point3d (double), 我们需要 CV_64F 类型
cv::Mat points_world_mat(points_world_vec.size(), 1, CV_64FC3);
// 将 vector 数据拷贝到 Mat 中
// 注意: Point3d 的内存布局与 CV_64FC3 Mat 的行向量兼容
for(size_t i = 0; i < points_world_vec.size(); ++i) {
    points_world_mat.at<cv::Point3d>(i, 0) = points_world_vec[i];
}
// 或者更简洁地直接创建 Mat 从 vector:
// cv::Mat points_world_mat(points_world_vec, true); // true 表示拷贝数据

// 2. 将旋转向量 rvec 转换为旋转矩阵 R (3x3)
cv::Mat R;
cv::Rodrigues(rvec, R);

// 3. 构建 3x4 的变换矩阵 M = [R | T]
// 确保 tvec 是 3x1 或 1x3 的 CV_64F 类型 Mat
if (tvec.rows != 3 || tvec.cols != 1) {
    if (tvec.rows == 1 && tvec.cols == 3) {
        tvec = tvec.t(); // 转置为 3x1
    } else {
        // 如果 tvec 不是 3x1 或 1x3，可能需要根据实际情况进行处理
        // 例如，如果它是 3x1 但类型不对，可以使用 convertTo
        // tvec.convertTo(tvec, CV_64F);
        std::cerr << "Warning: tvec is not 3x1 or 1x3. Assuming it's convertible to 3x1 CV_64F." << std::endl;
        if (tvec.total() == 3) {
             tvec = tvec.reshape(0, 3); // 重塑为 3x1
        } else {
            std::cerr << "Error: tvec cannot be reshaped to 3x1." << std::endl;
            return 1; // 退出或抛出错误
        }
    }
}
if (R.type() != CV_64F) R.convertTo(R, CV_64F);
if (tvec.type() != CV_64F) tvec.convertTo(tvec, CV_64F);


cv::Mat transform_matrix;
cv::hconcat(R, tvec, transform_matrix); // transform_matrix 是 3x4

// 4. 使用 cv::transform 进行坐标转换
cv::Mat points_camera_mat;
cv::transform(points_world_mat, points_camera_mat, transform_matrix);

// 5. (可选) 将结果从 cv::Mat 转换回 std::vector<cv::Point3d>
std::vector<cv::Point3d> points_camera_vec;
points_camera_vec.reserve(points_camera_mat.total());
for(int i = 0; i < points_camera_mat.rows; ++i) {
    // points_camera_mat 现在是 Nx1 的 CV_64FC3 矩阵
    points_camera_vec.push_back(points_camera_mat.at<cv::Point3d>(i, 0));
}


// 输出转换后的相机坐标系下的点
std::cout << "Points in Camera Coordinate System:" << std::endl;
for (const auto& p_camera : points_camera_vec) {
    std::cout << "(" << p_camera.x << ", " << p_camera.y << ", " << p_camera.z << ")" << std::endl;
}

// 如果您只需要 Mat 形式的结果，直接使用 points_camera_mat 即可
// std::cout << "Points in Camera Coordinate System (Mat):" << std::endl;
// std::cout << points_camera_mat << std::endl;

return 0; // 示例代码结束