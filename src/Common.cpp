/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#include "Common.h"

namespace DosuRay
{
	int pyrLevelsUsed = 5;
	int wG[PYR_LEVELS], hG[PYR_LEVELS];
	float fxG[PYR_LEVELS], fyG[PYR_LEVELS],
				cxG[PYR_LEVELS], cyG[PYR_LEVELS];

	float fxiG[PYR_LEVELS], fyiG[PYR_LEVELS],
				cxiG[PYR_LEVELS], cyiG[PYR_LEVELS];
  Eigen::Matrix3f KG[PYR_LEVELS], KiG[PYR_LEVELS];
	float weights[PYR_LEVELS];

	// Eigen::Vector3f* dI;				       //灰度图像[0];build label[1];point_label[2]
	// Eigen::Vector3f* dIp[PYR_LEVELS];	 //金字塔图像
	// float* absSquaredGrad[PYR_LEVELS]; //金字塔像素梯度


	// size_t kWidth = 1281;//像素宽
	// size_t kHeight = 1025;//像素高
	bool print_mode = false;
	int sample_size = 10; //直线离散化采样点数量
	double pickup_range = 500.0; //json文件提取建筑半径范围
	double tree_range = 200.0; //以某一相机位置半径范围构建区间树
	float iou_threhold = 0.8;

	int patternP[8][2] = {{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},
													 {0,0},		  {2,0},	   {-1,1},		{0,2}};
	int patternNum = 8;
	int pixelLabelBuild[kWidth*kHeight] = {0};
	int pixelLabelFace[kWidth*kHeight] = {0};

	void setCalib(int w, int h, const Eigen::Matrix3f &K)
	{
		wG[0] = w;
		hG[0] = h;
		KG[0] = K;
		fxG[0] = K(0,0);
		fyG[0] = K(1,1);
		cxG[0] = K(0,2);
		cyG[0] = K(1,2);
		KiG[0] = KG[0].inverse();
		fxiG[0] = KiG[0](0,0);
		fyiG[0] = KiG[0](1,1);
		cxiG[0] = KiG[0](0,2);
		cyiG[0] = KiG[0](1,2);

 		weights[0] = 0.1;
		for (int level = 1; level < pyrLevelsUsed; ++ level)
		{
			wG[level] = w >> level;
			hG[level] = h >> level;

			fxG[level] = fxG[level-1] * 0.5;
			fyG[level] = fyG[level-1] * 0.5;
			cxG[level] = (cxG[0] + 0.5) / ((int)1<<level) - 0.5;
			cyG[level] = (cyG[0] + 0.5) / ((int)1<<level) - 0.5;

			weights[level] = weights[level-1] * 2.0;
	  }

	}


}
