#include"ArmorDector.h"
#include<iostream>
#include<math.h>
#include<vector>
#include<string>

enum {
	WIDTH_GREATER_THAN_HEIGHT ,
	ANGLE_TO_UP
};

template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2) {
	return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

//获取两直线交点
template<typename ValType>
const cv::Point2f crossPointOf(const std::array<cv::Point_<ValType>, 2>& line1, const std::array<cv::Point_<ValType>, 2>& line2) {
	ValType a1 = line1[0].y - line1[1].y;
	ValType b1 = line1[1].x - line1[0].x;
	ValType c1 = line1[0].x * line1[1].y - line1[1].x * line1[0].y;

	ValType a2 = line2[0].y - line2[1].y;
	ValType b2 = line2[1].x - line2[0].x;
	ValType c2 = line2[0].x * line2[1].y - line2[1].x * line2[0].y;

	ValType d = a1 * b2 - a2 * b1;

	if (d == 0.0) {
		return cv::Point2f(FLT_MAX, FLT_MAX);
	} else {
		return cv::Point2f(float(b1 * c2 - b2 * c1) / d, float(c1 * a2 - c2 * a1) / d);
	}
}
inline const cv::Point2f crossPointOf(const cv::Vec4f& line1, const cv::Vec4f& line2) {
	const std::array<cv::Point2f, 2> line1_{ cv::Point2f(line1[2],line1[3]),cv::Point2f(line1[2] + line1[0],line1[3] + line1[1]) };
	const std::array<cv::Point2f, 2> line2_{ cv::Point2f(line2[2],line2[3]),cv::Point2f(line2[2] + line2[0],line2[3] + line2[1]) };
	return crossPointOf(line1_, line2_);
}

template<typename T1, typename T2>
const cv::Rect_<T1> scaleRect(const cv::Rect_<T1>& rect, const cv::Vec<T2, 2> scale, cv::Point_<T1> anchor = cv::Point_<T1>(-1, -1)) {
	//如果未指定anchor，则将anchor设为矩形中心
	if (anchor == cv::Point_<T1>(-1, -1)) {
		anchor = cv::Point_<T1>(rect.x + rect.width / 2, rect.y + rect.height / 2);
	}
	//计算缩放后矩形的左上角坐标
	cv::Point_<T1> tl((cv::Vec<T1, 2>(rect.tl()) - cv::Vec<T1, 2>(anchor)).mul(scale) + cv::Vec<T1, 2>(anchor));
	//计算缩放后矩形的大小
	cv::Size_<T1> size(rect.width * scale[0], rect.height * scale[1]);
	return cv::Rect_<T1>(tl, size);
}
//调整矩形角度
cv::RotatedRect& ArmorDector:: adjustRec(cv::RotatedRect& rec, const int mode) {
	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	//宽大于高的情况，将宽高数据互换,并且将矩形旋转90°
	if (WIDTH_GREATER_THAN_HEIGHT) {
		if (width < height) {
			std::swap(width, height);
			angle += 90.0;
		}
	}

	//确保角度在-90到90
	while (angle >= 90) {
		angle -= 180;
	}
	while (angle < -90) {
		angle += 180;
	}

	if (mode == ANGLE_TO_UP) {
		if (angle >= 45.0) {
			std::swap(width, height);
			angle -= 90.0;
		} else if (angle < -45.0) {
			std::swap(width, height);
			angle += 90.0;
		}
	}

	return rec;
}

ArmorBox::ArmorBox(const LightBar& l_light, const LightBar& r_light, const int armor_type, const cv::Mat& srcimg, const float& RotationScore, Armor_param param) {
	//左右灯条矩形轮廓
	light_pairs[0] = l_light.Ro_rec();
	light_pairs[1] = r_light.Ro_rec();

	//轮廓拓展
	cv::Size exLSize(int(light_pairs[0].size.width), int(light_pairs[0].size.height * 2));
	cv::Size exRSize(int(light_pairs[1].size.width), int(light_pairs[1].size.height * 2));
	cv::RotatedRect exLLight(light_pairs[0].center, exLSize, light_pairs[0].angle);
	cv::RotatedRect exRLight(light_pairs[1].center, exRSize, light_pairs[1].angle);

	//获取左灯条四点坐标，得到装甲板左上角和左下角坐标
	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point upper_l = pts_l[2];
	cv::Point lower_l = pts_l[3];

	//获取右灯条四点坐标，得到装甲板右上角和右下角坐标
	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r = pts_r[1];
	cv::Point2f lower_r = pts_r[0];

	vertex.resize(4);
	vertex[0] = upper_l;
	vertex[1] = upper_r;
	vertex[2] = lower_r;
	vertex[3] = lower_l;

	//设置装甲板类型
	type = armor_type;

	getFrontImg(srcimg);
	rotationScore = RotationScore;
	
	float normal_area = cv::contourArea(vertex) / param.area_normal_base;
	sizeScore = exp(normal_area);

	cv::Point2f srcImgCenter(srcimg.cols / 2, srcimg.rows / 2);
	float sightoffset = distance(srcImgCenter, crossPointOf(std::array<cv::Point2f, 2>{vertex[0], vertex[2]}, std::array<cv::Point2f, 2>{vertex[1], vertex[3]}));
	disScore = exp(-sightoffset / param.sight_offset_normal_base);

}

void ArmorBox::getFrontImg(const cv::Mat& grayImg) {
	const cv::Point2f&
		tl = vertex[0],
		tr = vertex[1],
		br = vertex[2],
		bl = vertex[3];

	int width, height;
	if (type == BIG) {
		width = 92;
		height = 50;
	} else {
		width = 50;
		height = 50;
	}

	//获取透视变换的参数坐标
	cv::Point2f src[4]{ cv::Vec2f(tl),cv::Vec2f(tr),cv::Vec2f(br),cv::Vec2f(bl) };
	cv::Point2f dst[4]{ cv::Point2f(0.0,0.0),cv::Point2f(width,0.0),cv::Point2f(width,height),cv::Point2f(0.0,height) };
	const cv::Mat perspMat = cv::getPerspectiveTransform(src, dst);
	cv::warpPerspective(grayImg, frontimg, perspMat, cv::Size(width, height));
}

ArmorDector::ArmorDector() {
	_flag = ARMOR_NO;
	roi = cv::Rect(cv::Point(0.0), Src_Img.size());
	_isTracking = false;
	//载入装甲板数字模板
	small_tem[0] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/1.jpg");
	small_tem[1] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/2.jpg");
	small_tem[2] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/3.jpg");
	small_tem[3] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/4.jpg");
	small_tem[4] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/5.jpg");
	small_tem[5] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/6.jpg");
	small_tem[6] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/7.jpg");
	small_tem[7] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/8.jpg");
	big_tem[0] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/11.jpg");
	big_tem[1] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/22.jpg");
	big_tem[2] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/33.jpg");
	big_tem[3] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/44.jpg");
	big_tem[4] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/55.jpg");
	big_tem[5] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/66.jpg");
	big_tem[6] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/77.jpg");
	big_tem[7] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/88.jpg");

#if defined SHOW_RESULT
	_debugWindowName = "debug_info";
#endif
}

ArmorDector::ArmorDector(const Armor_param& armorParam) {
	armor_param = armorParam;
	_flag = ARMOR_NO;
	roi = cv::Rect(cv::Point(0, 0), Src_Img.size());
	_isTracking = false;
	//载入装甲板数字模板
	small_tem[0] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/1.jpg");
	small_tem[1] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/2.jpg");
	small_tem[2] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/3.jpg");
	small_tem[3] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/4.jpg");
	small_tem[4] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/5.jpg");
	small_tem[5] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/6.jpg");
	small_tem[6] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/7.jpg");
	small_tem[7] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/8.jpg");
	big_tem[0] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/11.jpg");
	big_tem[1] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/22.jpg");
	big_tem[2] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/33.jpg");
	big_tem[3] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/44.jpg");
	big_tem[4] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/55.jpg");
	big_tem[5] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/66.jpg");
	big_tem[6] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/77.jpg");
	big_tem[7] = cv::imread("C:/Users/32380/Downloads/Template.zip/Template/88.jpg");
#if defined(SHOW_RESULT)
	_debugWindowName = "debug_info";
#endif
}

void ArmorDector::armor_init(const Armor_param& armorParam) {
	armor_param = armorParam;
}

void ArmorDector::loadimg(const cv::Mat& srcImg) {
	Src_Img = srcImg;
#if defined (SHOW_RESULT)
	_debugImg = srcImg.clone();
#endif
	cv::Rect imgBound = cv::Rect(cv::Point(0, 0), srcImg.size());
	if (_flag == ARMOR_LOCAL && _track != armor_param.max_track_num) {
		//计算目标装甲板的顶点包围矩形
		cv::Rect bRect = boundingRect(targetArmor.vertex) + roi.tl();
		bRect = scaleRect(bRect, cv::Vec2f(3, 2));//将中心为锚点放大2倍
		roi = bRect & imgBound;
		Roi_Img = Src_Img(roi).clone();
	} else {
		roi = imgBound;
		Roi_Img = Src_Img.clone();
		_track = 0;
	}
}

int ArmorDector::detect() {
	Armors.clear();
	std::vector<LightBar> lightInfos;

	//把一个3通道图像转换为3个通道图像
	std::vector<cv::Mat> channels;
	cv::split(Roi_Img, channels);
	if (EnemyColor == RED) {
		Gray_Img = channels.at(2) - channels.at(0);//red-blue
	} else {
		Gray_Img = channels.at(0) - channels.at(2);//blue-red
	}

	//图像预处理
	cv::Mat binBrightImg;
	cv::cvtColor(Roi_Img, Gray_Img, cv::COLOR_BGR2GRAY, 1);
	cv::threshold(Gray_Img, binBrightImg, armor_param.bright_threshold, 255, cv::THRESH_BINARY);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_BLACKHAT, cv::Size(3, 3));
	cv::dilate(binBrightImg, binBrightImg, element);
	//识别灯条并且初步匹配装甲板灯条
	FindLight(binBrightImg,lightInfos,channels);

	//对找到的装甲板进行筛选
	Armors.erase(std::remove_if(Armors.begin(), Armors.end(), [this](ArmorBox& i) {
		return !(i.IsArmorPattern(small_tem,big_tem));
		}), Armors.end());
	if (Armors.empty()) {
		targetArmor.clear();
		if (_flag == ARMOR_LOCAL) {
			return _flag = ARMOR_LOST;
		} else {
			return _flag = ARMOR_NO;
		}
	}

	for (auto& armor : Armors) {
		armor.finalScore = armor.sizeScore + armor.disScore + armor.disScore;
	}
	std::sort(Armors.begin(), Armors.end(), [](const ArmorBox& a, const ArmorBox& b) {
		return a.finalScore > b.finalScore;
		});
	targetArmor = Armors[0];
	_track++;
#if defined SHOW_RESULT
	std::vector<cv::Point> intVertex;
	for (const auto& point : targetArmor.vertex) {
		cv::Point fuckPoint = point;
		intVertex.emplace_back(fuckPoint);
	}
#endif
	return _flag = ARMOR_LOCAL;
}

const std::vector<cv::Point2f> ArmorDector::getArmorVertex() {
	std::vector<cv::Point2f> realVectex;
	for (int i = 0; i < 4; i++) {
		realVectex.emplace_back(cv::Point2f(targetArmor.vertex[i].x + roi.tl().x, targetArmor.vertex[i].y + roi.tl().y));
	}
	return realVectex;
}

int ArmorDector::getArmorType() const {
	return targetArmor.type;
}

#if defined SHOW_RESULT
void ArmorDector::showDebugImg() const {
	imshow(_debugWindowName, _debugImg);
}
#endif