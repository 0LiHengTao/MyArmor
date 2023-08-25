#include"pnp_solver.h"
#include <opencv2/calib3d.hpp>

PnPSolver::PnPSolver(
    const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs) {
    // Initialize camera_matrix_
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camera_matrix_.at<double>(i, j) = camera_matrix[i * 3 + j];
        }
    }

    // Initialize dist_coeffs_
    dist_coeffs_ = cv::Mat(1, dist_coeffs.size(), CV_64F);
    for (size_t i = 0; i < dist_coeffs.size(); ++i) {
        dist_coeffs_.at<double>(0, i) = dist_coeffs[i];
    }

    // Initialize armor_points
    constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

    small_armor_points.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
    small_armor_points.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
    small_armor_points.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
    small_armor_points.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

    large_armor_points.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
    large_armor_points.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
    large_armor_points.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
    large_armor_points.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const ArmorBox& armor, cv::Mat rvec, cv::Mat tvec) {
    std::vector<cv::Point2f> image_armor_points;
    //获取装甲板左右灯条的顶部和底部坐标
    cv::Point2f pl[4], pr[4];
    armor.light_pairs[0].points(pl);
    armor.light_pairs[1].points(pr);

    //左右灯条顶点和底部坐标填入图像坐标
    image_armor_points.emplace_back((pl[2] + pl[3]) / 2);
    image_armor_points.emplace_back((pl[0] + pl[1]) / 2);
    image_armor_points.emplace_back((pr[0] + pr[1]) / 2);
    image_armor_points.emplace_back((pr[2] + pr[3]) / 2);

    //角度解算
    auto object_points = armor.type == (Armor_type::SMALL) ? small_armor_points : large_armor_points;
    return cv::solvePnP(
        object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE
    );
}

float PnPSolver::calculate_distance_to_center(const cv::Point2f& image_point) {
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}
