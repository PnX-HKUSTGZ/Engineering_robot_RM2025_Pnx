#include <iostream>
#include <vector>
#include <cmath>    // For std::fabs
#include <limits>   // For std::numeric_limits

#include <pcl/point_types.h> // For pcl::PointXYZ

/**
 * @brief Calculates the distance from a point to a plane defined by a vector of coefficients.
 *
 * The plane is defined by coefficients (a, b, c, d) in the vector for the equation ax + by + cz + d = 0.
 * It assumes (a, b, c) is a normalized normal vector, so sqrt(a^2 + b^2 + c^2) ≈ 1.
 * The distance from point (x0, y0, z0) is |a*x0 + b*y0 + c*z0 + d|.
 *
 * @param point The 3D point (x0, y0, z0).
 * @param plane_coefficients A vector of doubles containing the plane coefficients {a, b, c, d}.
 *                           Assumed to have size 4 and (a, b, c) normalized.
 * @return The distance from the point to the plane. Returns NaN if coefficients vector is invalid.
 */
float calculatePointToPlaneDistance(
    const pcl::PointXYZ& point,
    const std::vector<double>& plane_coefficients)
{
    // Ensure the coefficients vector has the correct size for a plane (a, b, c, d)
    if (plane_coefficients.size() != 4) {
        std::cerr << "Error: Invalid number of plane coefficients ("
                  << plane_coefficients.size() << " instead of 4)." << std::endl;
        return std::numeric_limits<float>::quiet_NaN(); // Return NaN for invalid input
    }

    // Use double for intermediate calculations for better precision, cast to float at the end if needed.
    // However, since pcl::PointXYZ uses float, the precision might be limited by the input point anyway.
    // Let's use float for consistency with pcl::PointXYZ. If higher precision is needed,
    // consider taking Eigen::Vector3d as input instead of pcl::PointXYZ.
    float a = static_cast<float>(plane_coefficients[0]);
    float b = static_cast<float>(plane_coefficients[1]);
    float c = static_cast<float>(plane_coefficients[2]);
    float d = static_cast<float>(plane_coefficients[3]);

    float x0 = point.x;
    float y0 = point.y;
    float z0 = point.z;

    // The signed distance is a*x0 + b*y0 + c*z0 + d
    // Since (a, b, c) are assumed normalized, sqrt(a^2+b^2+c^2) is 1.
    // We take the absolute value for the non-negative distance.
    float signed_distance = a * x0 + b * y0 + c * z0 + d;
    float distance = std::fabs(signed_distance); // Use std::fabs for float

    return distance;
}

// --- Example Usage ---
int main() {
    // Define a plane: x + y + z - 1 = 0 (coeffs: [1/sqrt(3), 1/sqrt(3), 1/sqrt(3), -1/sqrt(3)])
    // Normalization factor = sqrt(1^2 + 1^2 + 1^2) = sqrt(3)
    double norm_factor = std::sqrt(3.0);
    std::vector<double> plane_coeffs;
    plane_coeffs.push_back(1.0 / norm_factor); // a
    plane_coeffs.push_back(1.0 / norm_factor); // b
    plane_coeffs.push_back(1.0 / norm_factor); // c
    plane_coeffs.push_back(-1.0 / norm_factor); // d

    // --- Example 1: Point on the plane (1, 0, 0) ---
    // x + y + z - 1 = 1 + 0 + 0 - 1 = 0. Distance should be 0.
    pcl::PointXYZ p1(1.0f, 0.0f, 0.0f);
    float dist1 = calculatePointToPlaneDistance(p1, plane_coeffs);
    std::cout << "Point (" << p1.x << "," << p1.y << "," << p1.z << ") to plane x+y+z-1=0 distance: " << dist1 << std::endl;
    // Expected: ~0.0

    // --- Example 2: Point (0, 0, 0) (origin) ---
    // Distance from origin to x+y+z-1=0 is |-1|/sqrt(3) = 1/sqrt(3) ≈ 0.577
    pcl::PointXYZ p2(0.0f, 0.0f, 0.0f);
    float dist2 = calculatePointToPlaneDistance(p2, plane_coeffs);
    std::cout << "Point (" << p2.x << "," << p2.y << "," << p2.z << ") to plane x+y+z-1=0 distance: " << dist2 << std::endl;
    // Expected: ~0.577

    // --- Example 3: Point (1, 1, 1) ---
    // 1 + 1 + 1 - 1 = 2. Distance should be |2|/sqrt(3) = 2/sqrt(3) ≈ 1.154
    pcl::PointXYZ p3(1.0f, 1.0f, 1.0f);
    float dist3 = calculatePointToPlaneDistance(p3, plane_coeffs);
    std::cout << "Point (" << p3.x << "," << p3.y << "," << p3.z << ") to plane x+y+z-1=0 distance: " << dist3 << std::endl;
    // Expected: ~1.154

     // --- Example 4: Point (0, 0, 5) ---
     // 0 + 0 + 5 - 1 = 4. Distance should be |4|/sqrt(3) = 4/sqrt(3) ≈ 2.309
    pcl::PointXYZ p4(0.0f, 0.0f, 5.0f);
    float dist4 = calculatePointToPlaneDistance(p4, plane_coeffs);
    std::cout << "Point (" << p4.x << "," << p4.y << "," << p4.z << ") to plane x+y+z-1=0 distance: " << dist4 << std::endl;
    // Expected: ~2.309


    // --- Example 5: Invalid coefficients ---
    std::vector<double> invalid_coeffs;
    invalid_coeffs.push_back(1.0); // Only one coefficient

    pcl::PointXYZ p5(0.0f, 0.0f, 0.0f);
    float dist5 = calculatePointToPlaneDistance(p5, invalid_coeffs);
    std::cout << "Point (" << p5.x << "," << p5.y << "," << p5.z << ") to invalid plane distance: " << dist5 << std::endl;
    // Expected: NaN

    return 0;
}