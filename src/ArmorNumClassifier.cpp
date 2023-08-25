#include"ArmorDector.h"

//用模板匹配的方法识别装甲板数组
double TemplateMatch(const cv::Mat& inputImage, const cv::Mat& templateImage, int matchMethod) {
	cv::Mat result;
	cv::matchTemplate(inputImage, templateImage, result, matchMethod);

	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
	return maxVal;
}

bool ArmorBox::IsArmorPattern(std::vector<cv::Mat>& small, std::vector<cv::Mat>& big) {

	std::vector<std::pair<int, double>> score;
	std::map<int, double> mp;
	cv::Mat regulatedImg = frontimg;

	for (int i = 0; i < 8; i++) {
		cv::Mat tepl = small[i];
		cv::Mat tepll = big[i];

		//模板匹配得分
		double value;
		//匹配小装甲
		value = TemplateMatch(regulatedImg, tepl, cv::TM_CCOEFF_NORMED);
		mp[i + 1] = value;
		score.push_back(std::make_pair(i + 1, value));
		//匹配大装甲
		value = TemplateMatch(regulatedImg, tepll, cv::TM_CCOEFF_NORMED);
		mp[i + 11] = value;
		score.push_back(std::make_pair(i + 11, value));
	}
	//对装甲板与所有模板的得分进行排序
	std::sort(score.begin(), score.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
		return a.second > b.second;
		});
	//装甲板中心位置
	cv::Point2f c=(vertex[0] + vertex[1] + vertex[2] + vertex[3]) / 4;
	//装甲板数字就是得分最高的位置
	int resultNum = score[0].first;
	//得分太低认为没有识别到
	if (score[0].second < 0.6) {
		return false;
	}
	//排除识别出错的可能
	if (type == SMALL) {
		if (resultNum > 10) {
			type = BIG;
		}
	}
	return true;
}