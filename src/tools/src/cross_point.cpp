#include "math/cross_point.hpp"

bool tools::getCrossPoint(
    const Eigen::Vector2f& line1_p1, const Eigen::Vector2f& line1_p2,
    const Eigen::Vector2f& line2_p1, const Eigen::Vector2f& line2_p2,
    Eigen::Vector2f& intersection
) {
    Eigen::Vector2f dir1 = line1_p2 - line1_p1;
    Eigen::Vector2f dir2 = line2_p2 - line2_p1;

    float denom = dir1.x() * dir2.y() - dir1.y() * dir2.x();

    if (fabs(denom) < 1e-6) {
        return false;
    }

    float t = ((line2_p1.x() - line1_p1.x()) * dir2.y() - 
               (line2_p1.y() - line1_p1.y()) * dir2.x()) / denom;

    intersection.x() = line1_p1.x() + t * dir1.x();
    intersection.y() = line1_p1.y() + t * dir1.y();

    return true;
}