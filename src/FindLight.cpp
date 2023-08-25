#include"ArmorDector.h"

int ArmorDector::FindLight(const cv::Mat& binBrightImg, std::vector<LightBar>& lightInfos, std::vector<cv::Mat> channels) {
	std::vector<std::vector<cv::Point>> lightContours;
	//找轮廓
	cv::findContours(binBrightImg.clone(), lightContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	//找到轮廓后开始遍历提取灯条
	for (int i = 0; i < lightContours.size(); i++) {
		//得到每个矩形的面积
		float lightContourArea = cv::contourArea(lightContours[i]);
		if (lightContourArea < armor_param.light_min_area) continue;//面积筛选
		cv::RotatedRect lightRec = cv::fitEllipse(lightContours[i]);//椭圆拟合区域得到外界矩形
		adjustRec(lightRec, 1);//矫正灯条
		if ((lightRec.size.width / lightRec.size.height) > armor_param.light_max_ratio || (lightContourArea / lightRec.size.area())) continue;//宽高比筛选
		//对灯条比例进行略微放大
		lightRec.size.width *= armor_param.light_Color_detect_extend_ratio;
		lightRec.size.height *= armor_param.light_Color_detect_extend_ratio;
		cv::Rect lightRect = lightRec.boundingRect();
		const cv::Rect srcBound(cv::Point(0, 0), Roi_Img.size());
		lightRect &= srcBound;
		//颜色通道相减后己方灯条直接过滤不需要判断颜色，直接保存灯条
		lightInfos.push_back(LightBar(lightRec));
	}
	//没找到灯条返回未找到
	if (lightInfos.empty()) {
		return _flag = ARMOR_NO;
	}
	std::sort(lightInfos.begin(), lightInfos.end(), [](const LightBar& ld1, const LightBar& ld2) {
		//Lambda函数作为sort函数的cmp函数
		return ld1.center.x < ld2.center.x;
		});
	for (size_t i = 0; i < lightInfos.size(); i++) {
		//遍历灯条进行匹配
		for (size_t j = i + 1; j < lightInfos.size(); j++) {
			const LightBar& leftLight = lightInfos[i];
			const LightBar& rightLight = lightInfos[j];

			//角差
			float angleDiff_ = abs(leftLight.angle - rightLight.angle);
			//长度差比率
			float LenDiff_ratio = abs(leftLight.length - rightLight.length);
			//筛选
			if ((angleDiff_ > armor_param.light_max_angle_diff) || (LenDiff_ratio > armor_param.light_max_height_diff_ratio)) {
				continue;
			}

			//左右灯条相距距离
			float dis = std::sqrt(std::pow((leftLight.center.x) - (rightLight.center.x), 2) + std::pow((leftLight.center.y) - (rightLight.center.y), 2));
			//左右灯条长度平均值
			float meanLen = (leftLight.length + rightLight.length) / 2;
			//左右灯条中心点y的差值
			float yDiff = abs(leftLight.center.y - rightLight.center.y);
			//y差比率
			float yDiff_ratio = yDiff / meanLen;
			//左右灯条中心点x的差值
			float xDiff = abs(leftLight.center.x - rightLight.center.x);
			//x差比率
			float xDiff_ratio = xDiff / meanLen;
			//相距距离与灯条长度比值
			float ratio = dis / meanLen;
			//筛选
			if (yDiff_ratio > armor_param.light_max_y_diff_ratio ||
				xDiff_ratio<armor_param.light_max_x_diff_ratio ||
				ratio>armor_param.armor_max_aspect_ratio_ ||
				ratio < armor_param.armor_min_aspect_ratio_) {
				continue;
			}
			
			//确定装甲板类型
			int armorType = ratio > armor_param.armor_big_armor_ratio ? BIG : SMALL;
			float ratiOff = (armorType == BIG) ? std::max(armor_param.armor_big_armor_ratio - ratio, float(0)) : std::max(armor_param.armor_small_armor_ratio - ratio, float(0));
			float yOff = yDiff / meanLen;
			float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
			//得到装甲板
			ArmorBox armor(leftLight, rightLight, armorType, channels.at(1), rotationScore, armor_param);
			Armors.emplace_back(armor);
			break;
		}
	}
	//没匹配到装甲板就返回没有找到
	if (Armors.empty()) {
		return _flag = ARMOR_NO;
	}

}