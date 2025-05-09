#include <iostream>
#include <vector>
#include <cmath>    // For std::sqrt, std::fabs
#include <limits>   // For std::numeric_limits

// For Eigen vectors and matrix operations (double precision)
#include <Eigen/Dense>

/**
 * @brief Determines the plane defined by three given points.
 *        Input points are Eigen::Vector3d (Eigen::Matrix<double, 3, 1>).
 *
 * The plane equation is ax + by + cz + d = 0, where (a, b, c) is the normalized normal vector.
 *
 * @param p1 The first point (Eigen::Vector3d).
 * @param p2 The second point (Eigen::Vector3d).
 * @param p3 The third point (Eigen::Vector3d).
 * @param epsilon Tolerance for checking if points are collinear (normal vector is close to zero).
 * @return A std::vector<double> containing the coefficients {a, b, c, d} if successful (size 4).
 *         Returns an empty std::vector<double> if the three points are collinear.
 */
std::vector<double> determinePlaneFromThreePoints(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3,
    double epsilon = 1e-9) // Using double precision epsilon
{
    // Calculate two vectors on the plane using double precision
    Eigen::Vector3d v12 = p2 - p1;
    Eigen::Vector3d v13 = p3 - p1;

    // Calculate the normal vector as the cross product (double precision)
    Eigen::Vector3d normal = v12.cross(v13);

    // Check if the normal vector is close to zero (points are collinear) using double precision norm
    if (normal.norm() < epsilon) {
        std::cerr << "Error: The three points are collinear. Cannot determine a unique plane." << std::endl;
        return std::vector<double>(); // Return empty vector to indicate failure
    }

    // Normalize the normal vector (double precision)
    Eigen::Vector3d normalized_normal = normal.normalized();

    // Calculate the d coefficient: d = -(a*x + b*y + c*z) for any point (x,y,z) on the plane
    // Using p1 and double precision dot product: d = -(normalized_normal . p1)
    double d = -normalized_normal.dot(p1);

    // Store normalized a, b, c coefficients and d into a vector<double>
    std::vector<double> coefficients;
    coefficients.push_back(normalized_normal.x()); // a
    coefficients.push_back(normalized_normal.y()); // b
    coefficients.push_back(normalized_normal.z()); // c
    coefficients.push_back(d);                    // d

    return coefficients; // Return the vector of coefficients
}

// --- Example Usage ---
int main() {
    // --- Example 1: Three non-collinear points ---
    // Points: (1, 0, 0), (0, 1, 0), (0, 0, 1)
    // These points lie on the plane x + y + z - 1 = 0
    Eigen::Vector3d p1(1.0, 0.0, 0.0);
    Eigen::Vector3d p2(0.0, 1.0, 0.0);
    Eigen::Vector3d p3(0.0, 0.0, 1.0);

    std::vector<double> plane_coeffs = determinePlaneFromThreePoints(p1, p2, p3);

    if (!plane_coeffs.empty()) {
        std::cout << "Plane from points (" << p1.transpose() << "), "
                  << "(" << p2.transpose() << "), "
                  << "(" << p3.transpose() << ") Coefficients: ";
        std::cout << "a=" << plane_coeffs[0]
                  << ", b=" << plane_coeffs[1]
                  << ", c=" << plane_coeffs[2]
                  << ", d=" << plane_coeffs[3] << std::endl;
        // Expected (approximately, due to normalization):
        // Normal should be proportional to (1, 1, 1)
        // Normalized normal is (1/sqrt(3), 1/sqrt(3), 1/sqrt(3)) ≈ (0.577, 0.577, 0.577)
        // d = -(0.577*1 + 0.577*0 + 0.577*0) = -0.577
        // So coeffs should be roughly [0.577, 0.577, 0.577, -0.577] (or the negative of these)

    } else {
        std::cout << "Failed to determine plane for example 1." << std::endl;
    }

    std::cout << "\n";

    // --- Example 2: Three collinear points ---
    // Points: (0, 0, 0), (1, 0, 0), (2, 0, 0) - all on the X axis
    Eigen::Vector3d p4(0.0, 0.0, 0.0);
    Eigen::Vector3d p5(1.0, 0.0, 0.0);
    Eigen::Vector3d p6(2.0, 0.0, 0.0);

    std::vector<double> plane_coeffs_collinear = determinePlaneFromThreePoints(p4, p5, p6);

     if (!plane_coeffs_collinear.empty()) {
        std::cout << "Plane from collinear points (" << p4.transpose() << "), "
                  << "(" << p5.transpose() << "), "
                  << "(" << p6.transpose() << ") Coefficients: ";
        std::cout << "a=" << plane_coeffs_collinear[0]
                  << ", b=" << plane_coeffs_collinear[1]
                  << ", c=" << plane_coeffs_collinear[2]
                  << ", d=" << plane_coeffs_collinear[3] << std::endl;
        // Should not reach here
    } else {
        std::cout << "Correctly failed to determine plane for example 2 (collinear points)." << std::endl;
    }

    return 0;
}