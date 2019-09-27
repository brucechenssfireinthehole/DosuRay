/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef FRAMESHELL_H
#define FRAMESHELL_H

//common support: PCL/Eigen...
#include "Common.h"

using namespace std;

namespace DosuRay
{
  struct FrameShell
  {
    Eigen::Vector3f* dI;				       //color[0];dx[1];dy[2]
    Eigen::Vector3f* dIp[PYR_LEVELS];	 //金字塔图像
    float* absSquaredGrad[PYR_LEVELS];  //金字塔像素梯度值
    void makeImages(cv::Mat query); //建立图像金字塔
  };
}//namespace FrameShell
#endif
