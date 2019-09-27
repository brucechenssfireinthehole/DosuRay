/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef COMMON_H
#define COMMON_H

//sys
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <list>
#include <algorithm>
#include <vector>
#include <string>
#include <time.h>
#include <unordered_map>
//support boost
#include <boost/format.hpp>
#include <boost/program_options.hpp>
//suport eigen
#include <Eigen/Core>
#include <Eigen/Dense>
//suport pcl
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
//support opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef M_PI
  #define M_PI 3.1415926535898
#endif

#define EPS  0.00001
#define EARTH_RADIUS 6371393
#define CLOCK_PER_SEC ((clock_t)1000000)
#define PYR_LEVELS 6 //pyramid layer 6
//uncommit WRITE_PFM if you have a viewer
//#define WRITE_PFM 1

using namespace std;
using namespace cv;
//***if use commad line input please uncommit it***
namespace bpo = boost::program_options;

//typedef
typedef Eigen::Vector4f Point4f_;
typedef Eigen::Vector4i Point4i_;
typedef Eigen::Vector3d Pointd_;
typedef Eigen::Vector3f Pointf_;
typedef Eigen::Vector3i Pointi_;
typedef Eigen::Vector2d Point2d_;
typedef Eigen::Vector2f Point2f_;
typedef Eigen::Vector2i Point2i_;
typedef pcl::PointXYZI PointT_;

namespace DosuRay
{
  //parameter setting
  const size_t kWidth = 300;//像素宽 原始尺寸 1281
  const size_t kHeight = 250;//像素高 原始尺寸 1025
  extern bool print_mode;
  extern int sample_size; //直线离散化采样点数量
  extern double pickup_range; //json文件提取建筑半径范围
  extern double tree_range; //以某一相机位置半径范围构建区间树
  extern float iou_threhold;// 建筑bounding box检查的交并比阈值
  //图像金字塔相关
  extern int pyrLevelsUsed; //实际使用的金字塔层数
  extern int wG[PYR_LEVELS], hG[PYR_LEVELS];
  extern float fxG[PYR_LEVELS], fyG[PYR_LEVELS],
               cxG[PYR_LEVELS], cyG[PYR_LEVELS];
  extern float fxiG[PYR_LEVELS], fyiG[PYR_LEVELS],
               cxiG[PYR_LEVELS], cyiG[PYR_LEVELS];
  extern Eigen::Matrix3f KG[PYR_LEVELS],KiG[PYR_LEVELS];
  extern Eigen::Vector3f* dI;				       //灰度图像[0];build label[1];point_label[2]
	extern Eigen::Vector3f* dIp[PYR_LEVELS];	 //金字塔图像
	extern float* absSquaredGrad[PYR_LEVELS]; //金字塔像素梯度
  extern float weights[PYR_LEVELS];   //每层金字塔权重值
  //点的可视性判断pattern
  extern int patternP[8][2];
  extern int patternNum;

  extern int pixelLabelBuild[kWidth*kHeight];
  extern int pixelLabelFace[kWidth*kHeight];
  // unordered_map<int, pair<int,int>> pixelmap;
  /*
  //replace stod()
  inline double strToDouble(string str)
  {
    int size=str.size();
    if(size==0)
    {
        return 0;
    }
    int pos=0;
    double res=0.0;
    for(int i=0;i<size;i++)
    {
        if(str[i]=='.')
        {
            pos=i;
            break;
        }
    }
    for(int i=0;i<size;i++)
    {
        if(i<pos)
        {
            res+=(str[i]-'0')*pow(10,pos-i-1);
        }
        if(i>pos)
        {
            res+=(str[i]-'0')*pow(10,pos-i);
        }
    }
    return res;
  }
  */

  inline vector<Point2d_> strToPoint2d(string str)
  {
    int size=str.size();
    int left=0;
    int right = size-1;
    vector<Point2d_> res;

    for(int i=0;i<size;i++)
    {
        if(str[i]=='(')
        {
          if(left<i)
          left = i;
        }
        if(str[i]==')')
        {
          if(right>i)
          right = i;
        }
    }
    string temp = str.substr(left+1,right-left-1);
    stringstream s(temp);

    string temp_s;
    while (getline(s, temp_s,','))
    {
      stringstream ss(temp_s);
      string temp_ss;
      vector<double> pts;
      while(ss>>temp_ss){
        // cout<<temp_ss<<endl;
        pts.push_back(stod(temp_ss));
        // cout<<to_string(pts.back())<<endl;
        cout<<setiosflags(ios::fixed)<<setprecision(10)<<pts.back()<<endl;
      }
      if(pts.size()!=2){
        continue;
      }
      else{
        Point2d_ pt(pts[0],pts[1]);
        res.push_back(pt);
      }
    }

    return res;
  }

  inline void lonlatToXY(Point2d_ &lonlat, double* lonlat_origin)
  {
    lonlat(0) = (lonlat(0)- lonlat_origin[0]) * (M_PI /180.0) * EARTH_RADIUS;
    lonlat(1) = (lonlat(1) -lonlat_origin[1]) * (M_PI /180.0) * EARTH_RADIUS;
  }

  void setCalib(int w, int h, const Eigen::Matrix3f &K);
  void makeImages(cv::Mat query);

}
#endif //Common.h
