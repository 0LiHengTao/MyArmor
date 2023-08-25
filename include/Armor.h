#pragma once

#include<opencv2/opencv.hpp>
#include<opencv2/ml.hpp>
#include<math.h>
#include<map>
#include<array>
#include<vector>
#include<string>

#define SHOW_RESULT

enum Color
{
	BLUE = 0 ,
	GREEN = 1 ,
	RED = 2 
};
//枚举装甲板型号大小
enum Armor_type
{
	UNKNOWN = 0 ,
	SMALL = 1 ,
	BIG = 2 ,
	MINI = 3 ,
	LARGE =4 ,

};

//装甲板参数
struct Armor_param
{
	//阈值
	int bright_threshold;//亮度二值化阈值
	int color_threshold;//通道相减判断红蓝所用的二值化阈值

	float light_min_area;//灯条允许的最小面积
	float light_max_angle;//灯条允许的最大偏角
	float light_min_size;//灯条允许的最小尺寸
	float light_max_ratio;//灯条允许的宽高比率

	float light_max_angle_diff;//两灯条允许的最大角度差值
	float light_max_height_diff_ratio;//两灯条允许的高度差比值
	float light_max_y_diff_ratio;//灯条间最大垂直距离差比例
	float light_max_x_diff_ratio;//灯条间最小水平距离差比例
	float light_Color_detect_extend_ratio;//灯条矩形轮廓拓展比例

	float armor_big_armor_ratio; //大装甲板宽高比
	float armor_small_armor_ratio; //小装甲板宽高比
	float armor_min_aspect_ratio_;//最小装甲板纵横比
	float armor_max_aspect_ratio_;//最大装甲板纵横比

	float sight_offset_normal_base;
	float area_normal_base;
	int enemy_color;//敌方颜色
	int max_track_num;//最大的追踪数量

	Armor_param() {
		bright_threshold = 60;
		color_threshold = 80;

		light_min_area = 50;
		light_max_angle = 45.0;
		light_min_size = 5.0;
		light_max_ratio = 1.0;

		light_max_angle_diff = 6;
		light_max_height_diff_ratio = 0.2;
		light_max_y_diff_ratio = 0.5;
		light_max_x_diff_ratio = 4.5;
		light_Color_detect_extend_ratio = 1.1;

		armor_big_armor_ratio = 3.2;
		armor_small_armor_ratio = 2;
		armor_min_aspect_ratio_ = 1.0;
		armor_max_aspect_ratio_ = 5.0;

		
		enemy_color = BLUE;
		max_track_num = 100;
		
	}
};

//灯条类
class LightBar:public cv::RotatedRect
{
public:
	LightBar() {};
	/*
	LightBar(const cv::RotatedRect& light) {
		width = light.size.width;
		length = light.size.height;
		angle = light.angle;
		area = light.size.area();
		center = light.center;
	}
	*/
	explicit LightBar(cv::RotatedRect box) : cv::RotatedRect(box) {
		cv::Point2f p[4];
		box.points(p);
		std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
		top = (p[0] + p[1]) / 2;
		bottom = (p[2] + p[3]) / 2;
		length = cv::norm(top - bottom);
		width = cv::norm(p[0] - p[1]);

		angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
		angle = angle / CV_PI * 180;
	}

	//=号运算符重载
	const LightBar& operator =(const LightBar& ld)
	{
		this->width = ld.width;
		this->length = ld.length;
		this->center = ld.center;
		this->angle = ld.angle;
		this->area = ld.area;
		this->bottom = ld.bottom;
		this->top = ld.top;
		return *this;
	}

	//获得灯条旋转矩形轮廓
	cv::RotatedRect Ro_rec() const {
		return cv::RotatedRect(center, cv::Size2f(width, length), angle);//拟合旋转矩形轮廓函数RotatedRect(),返回一个RotatedRect类
	}

public:
	float width;//灯条宽度
	float length;//灯条长度
	float angle;//灯条轮廓旋转矩形的角度
	float area;//矩形轮廓大小
	cv::Point2f center;//灯条中心坐标
	cv::Point2f bottom, top;

};

//装甲板类--装甲板特征
class ArmorBox
{
public:
	ArmorBox();
	ArmorBox(const LightBar& l_light, const LightBar& r_light, const int armor_type, const cv::Mat& srcimg, const float& rotationScore, Armor_param param);//有参构造初始化对象
	void clear() {
		sizeScore = 0;
		disScore = 0;
		rotationScore = 0;
		finalScore = 0;
		for (int i = 0; i < vertex.size(); i++) {
			vertex[i] = cv::Point2f(0, 0);
		}
		type = UNKNOWN;
	}
	void getFrontImg(const cv::Mat& GrayImg);//获取透视变换后的图像
	bool IsArmorPattern(std::vector<cv::Mat>&small,std::vector<cv::Mat>&big);//装甲板上面的数字匹配
public:
	float sizeScore;
	float disScore;
	float rotationScore;
	float finalScore;
	std::array<LightBar, 2>lightLR;//左右两个灯条
	std::array<cv::RotatedRect, 2> light_pairs;//两个灯条的旋转矩形轮廓（数组内存储左，右灯条）
	std::vector<cv::Point2f> vertex;//装甲板的四个顶点(包含了灯条)
	cv::Mat frontimg;//透视变换后的图像
	int type;//装甲板类型
};

