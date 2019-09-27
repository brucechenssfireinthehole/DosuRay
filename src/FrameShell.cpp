/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/
#include "FrameShell.h"

namespace DosuRay
{
  //query : gray img
  void FrameShell::makeImages(cv::Mat query)
  {
    //mat to float*
    int cols = query.cols;
    int rows = query.rows;
    float color[cols*rows];
    for (int i = 0; i < rows; i++)
	  {
      for (int j = 0; j < cols; j++)
		  {
        if(query.channels()==1){
          color[i*cols+j] = (float)query.at<uchar>(i,j);
        }
        else if(query.channels()==3){
          cv::Vec3b pix = query.at<cv::Vec3b>(i,j);
          uchar B = pix[0];
          uchar G = pix[1];
          uchar R = pix[2];
          color[i*cols+j] = R*0.299+G*0.587+B*0.114;
        }

		  }
		}

    for(int i=0;i<pyrLevelsUsed;i++)
    {
      dIp[i] = new Eigen::Vector3f[wG[i]*hG[i]];
      absSquaredGrad[i] = new float[wG[i]*hG[i]];
    }
    dI = dIp[0];
    // make d0
    int w=wG[0];
    int h=hG[0];
    for(int i=0;i<w*h;i++)
      dI[i][0] = color[i];  //pass image color, notice dI is a pointer.

    for(int lvl=0; lvl<pyrLevelsUsed; lvl++)
    {
      int wl = wG[lvl], hl = hG[lvl];
      Eigen::Vector3f* dI_l = dIp[lvl];

      float* dabs_l = absSquaredGrad[lvl];
      if(lvl>0)
      {
        int lvlm1 = lvl-1;
        int wlm1 = wG[lvlm1];
        Eigen::Vector3f* dI_lm = dIp[lvlm1];


        for(int y=0;y<hl;y++)
          for(int x=0;x<wl;x++)
          {
            dI_l[x + y*wl][0] = 0.25f * (dI_lm[2*x   + 2*y*wlm1][0] +
                          dI_lm[2*x+1 + 2*y*wlm1][0] +
                          dI_lm[2*x   + 2*y*wlm1+wlm1][0] +
                          dI_lm[2*x+1 + 2*y*wlm1+wlm1][0]);
          }
      }

      for(int idx=wl;idx < wl*(hl-1);idx++)
      {
        float dx = 0.5f*(dI_l[idx+1][0] - dI_l[idx-1][0]);  //calculate gradient deltax/deltay
        float dy = 0.5f*(dI_l[idx+wl][0] - dI_l[idx-wl][0]);


        if(!std::isfinite(dx)) dx=0; //in case the val is too big.
        if(!std::isfinite(dy)) dy=0;

        dI_l[idx][1] = dx; //pass the color gradient info, as well as color intensity;
        dI_l[idx][2] = dy; //the order in dI_l[w*d][3] is dI_l[_][color,dx,dy]


        dabs_l[idx] = sqrt(dx*dx+dy*dy);

        // if(setting_gammaWeightsPixelSelect==1 && HCalib!=0)
        // {
        // 	float gw = HCalib->getBGradOnly((float)(dI_l[idx][0]));
        // 	dabs_l[idx] *= gw*gw;	// convert to gradient of original color space (before removing response).
        // }
      }
    }
  }
}
