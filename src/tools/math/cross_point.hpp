#include <Eigen/Eigen>
#include <cmath>

namespace tools {
bool getCrossPoint(
    const Eigen::Vector2f& line1_p1, const Eigen::Vector2f& line1_p2,
    const Eigen::Vector2f& line2_p1, const Eigen::Vector2f& line2_p2,
    Eigen::Vector2f& intersection
);

}