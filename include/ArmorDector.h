#pragma once
#include"Armor.h"


class ArmorDector
{
public:
	ArmorDector();
	ArmorDector(const Armor_param& armor_param);
	~ArmorDector() {};
	void armor_init(const Armor_param& armor_param);//填入参数
	void loadimg(const cv::Mat& srcimg);//加载图像并且设置跟踪区域
	cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode);
	int FindLight(const cv::Mat& binBrightImg ,std::vector<LightBar>& lightInfos, std::vector<cv::Mat> channels);//传入一个预处理好的图像， 寻找灯条
	int detect();//装甲板自瞄算法，返回一个代表装甲板类型的枚举结果
	const std::vector<cv::Point2f> getArmorVertex();//获取装甲板的四个顶点坐标
	int getArmorType() const;//返回装甲板类型 0为小装甲板你，1为大装甲板
	void setEnemyColor(int enemyColor) {
		EnemyColor = enemyColor;
		SelfColor = enemyColor ==BLUE ? RED : BLUE;//与地方颜色相反
	}
public:
	enum ArmorFlag {
		ARMOR_NO = 0,	
		ARMOR_LOST = 1,		 
		ARMOR_GLOBAL = 2,	
		ARMOR_LOCAL = 3		
	};

#if defined(SHOW_RESULT)
	void showDebugImg() const;
#endif 
private:
	Armor_param armor_param;
	int EnemyColor;
	int SelfColor;
	cv::Rect roi;
	cv::Mat Roi_Img;
	cv::Mat Src_Img;
	cv::Mat Gray_Img;
	int _track;//跟踪计数

	std::vector<ArmorBox> Armors;
	//装甲板数字模板
	std::vector<cv::Mat> small_tem;
	std::vector<cv::Mat> big_tem;

	ArmorBox targetArmor;//目标装甲板

	int _flag;
	bool _isTracking;//标志和跟踪状态




#if defined(SHOW_RESULT)
	std::string _debugWindowName;
	cv::Mat _debugImg;
#endif

};
