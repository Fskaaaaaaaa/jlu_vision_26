#include <opencv2/core.hpp>
#include <vector>

namespace auto_buff {
// TODO: 对比官方尺寸，确定符的objpoints
// 写一个“传入BuffIndex返回组合后的世界点”的方法

// Rune object points
// r_tag, bottom_left, top_left, top_right, bottom_right
inline const std::vector<cv::Point3f> SINGLE_BLADE_OBJ_POINTS{
    cv::Point3f(0, 0, 0) / 1000,         cv::Point3f(0, -541.5, 186) / 1000,
    cv::Point3f(0, -858.5, 160) / 1000,  cv::Point3f(0, -858.5, -160) / 1000,
    cv::Point3f(0, -541.5, -186) / 1000,
};

} // namespace auto_buff
