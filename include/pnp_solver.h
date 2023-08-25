#pragma once
#include"ArmorDector.h"

class PnPSolver
{
public:
	PnPSolver(
		//接受相机矩阵和畸变系数
		const std::array<double, 9>& camera_matrix,
		const std::vector<double>& distortion_coefficients);


	//获取三维坐标,用于解决pnp问题，通过给定的装甲板信息计算旋转向量rvec和平移向量tvec
	bool solvePnP(const ArmorBox& armor, cv::Mat rvec, cv::Mat tvec);
	//计算装甲板中心到图像中心距离
	float calculate_distance_to_center(const cv::Point2f& image_point);
private:
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

	static constexpr float SMALL_ARMOR_WIDTH = 135;
	static constexpr float SMALL_ARMOR_HEIGHT = 55;
	static constexpr float LARGE_ARMOR_WIDTH = 225;
	static constexpr float LARGE_ARMOR_HEIGHT = 55;

	std::vector<cv::Point3f> small_armor_points;
	std::vector<cv::Point3f> large_armor_points;
};
