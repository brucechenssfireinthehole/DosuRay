/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#include "DosuRay.h"
#include "DataReader.h"
#include "FrameShell.h"

using namespace DosuRay;

string inputdir = "/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/";
// double origin_lon_lat[2] ={-73.999776,40.720681};//TODO
double origin_lon_lat[2] ={-74.008975,40.740187};//TODO

std::ostream& operator <<(std::ostream& stream, const Color& c)
{
  stream <<'('<< c.m_r <<','<< c.m_g <<','<< c.m_b <<')';
  return stream;
}

std::ostream& operator <<(std::ostream& stream, const Eigen::Vector3f& v)
{
  stream<<'['<<v(0)<<','<<v(1)<<','<<v(2)<<']';
  return stream;
}

struct camera_related
{
  Eigen::Matrix4f T;
  Eigen::Matrix3f K;
  float m_fieldOfViewInDegrees1;
  float m_fieldOfViewInDegrees2;
  Pointf_ m_origin;
  Eigen::Vector3f m_target;
  Eigen::Vector3f m_targetUpDirection;
  camera_related(float fieldOfViewInDegrees1,
                 float fieldOfViewInDegrees2,
                 const Pointf_& origin,
                 const Eigen::Vector3f& target,
                 const Eigen::Vector3f& targetUpDirection)
                 : m_fieldOfViewInDegrees1(fieldOfViewInDegrees1),
                   m_fieldOfViewInDegrees2(fieldOfViewInDegrees2),
                   m_origin(origin),
                   m_target(target),
                   m_targetUpDirection(targetUpDirection)
  {
    T =  T_W_C(origin, target, targetUpDirection);
    K = K_local(fieldOfViewInDegrees1,fieldOfViewInDegrees2);
  }

  Eigen::Matrix3f K_local(float fieldOfViewInDegrees1, float fieldOfViewInDegrees2)
  {
    Eigen::Matrix3f result = Eigen::Matrix3f::Identity();
    float tanFov1 = std::tan(fieldOfViewInDegrees1 * M_PI / (2*180.0));
    // float tanFov2 = tanFov1 *((float)kHeight / (float)kWidth);
    float tanFov2 = std::tan(fieldOfViewInDegrees2 * M_PI / (2*180.0));
    result(0,0) = kWidth / (2.0*tanFov1);
    result(1,1) = kHeight / (2.0*tanFov2);
    result(0,2) = kWidth / 2;
    result(1,2) = kHeight / 2;
    return result;
  }

  Eigen::Matrix4f T_W_C(const Pointf_& origin,
                        const Eigen::Vector3f& target,
                        const Eigen::Vector3f& targetUpDirection)
  {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();

    Eigen::Vector3f temp1 = target-origin;
    temp1.normalize();
    Eigen::Vector3f forward = temp1;
    Eigen::Vector3f temp2 = forward.cross(targetUpDirection);
    temp2.normalize();
    Eigen::Vector3f right = temp2;
    Eigen::Vector3f temp3 = forward.cross(right);
    temp3.normalize();
    Eigen::Vector3f down = temp3;
    result(0,0) = right(0);
    result(1,0) = right(1);
    result(2,0) = right(2);
    result(0,1) = down(0);
    result(1,1) = down(1);
    result(2,1) = down(2);
    result(0,2) = forward(0);
    result(1,2) = forward(1);
    result(2,2) = forward(2);

    result(0,3) = origin(0);
    result(1,3) = origin(1);
    result(2,3) = origin(2);

    return result.inverse(); // world to cam
  }
};

Ray makeCameraRay(float fieldOfViewInDegrees1,
                  float fieldOfViewInDegrees2,
                  const Pointf_& origin,
                  const Eigen::Vector3f& target,
                  const Eigen::Vector3f& targetUpDirection,
                  float xScreenPos0To1,
                  float yScreenPos0To1)
{
  Eigen::Vector3f temp1 = target-origin;
  temp1.normalize();
  Eigen::Vector3f forward = temp1;
  Eigen::Vector3f temp2 = forward.cross(targetUpDirection);
  temp2.normalize();
  Eigen::Vector3f right = temp2;
  Eigen::Vector3f temp3 = forward.cross(right);
  temp3.normalize();
  Eigen::Vector3f down = temp3;

  float tanFov1 = std::tan(fieldOfViewInDegrees1 * M_PI / (2*180.0));
  // float tanFov2 = tanFov1 * ((float)kHeight / (float)kWidth);
  float tanFov2 = std::tan(fieldOfViewInDegrees2 * M_PI / (2*180.0));
  Ray ray;

  //initial ray
  ray.m_origin = origin;
  ray.m_direction = forward + right * 2.0 * (xScreenPos0To1 -0.5) * tanFov1 + down * 2 * (yScreenPos0To1 -0.5) *tanFov2;
  Eigen::Vector3f tempp =ray.m_direction;
  tempp.normalize();
  ray.m_direction = tempp;
  float angle = (atan2(tempp(1),tempp(0))+M_PI) * 180.0 /M_PI;
  ray.m_angle = angle;
  return ray;
}

vector<Point2i_> do_sample(Pointf_ w1, Pointf_ w2, camera_related camera)
{
  vector<Point2i_> result;
  Eigen::Vector4f w1_, w2_;
  for(int i=0;i<3;i++)
  {
    w1_(i) = w1(i);
    w2_(i) = w2(i);
  }
  w1_(3) = 1.0;
  w2_(3) = 1.0;

  Eigen::Vector4f c1 = camera.T * w1_;
  Eigen::Vector4f c2 = camera.T * w2_;
  // cout<<c1.transpose()<<" *** "<<c2.transpose()<<endl;
  Eigen::Vector3f c1_ = c1.head(3);
  Eigen::Vector3f c2_ = c2.head(3);
  c1_ = c1_ / c1_(2);
  c2_ = c2_ / c2_(2);

  Pointf_ p1, p2;
  Point2i_ p1_, p2_;
  p1 = camera.K * c1_;
  p2 = camera.K * c2_;
  p1_(0) = (int)p1(0);
  p1_(1) = (int)p1(1);
  p2_(0) = (int)p2(0);
  p2_(1) = (int)p2(1);
  result.push_back(p1_);
  for (int i=1;i<sample_size-1;i++)
  {
    Point2i_ p_temp;
    p_temp(0) = (int) (p1(0)+(i* (p2(0)-p1(0)) / (float)(sample_size-1)));
    p_temp(1) = (int) (p1(1)+(i* (p2(1)-p1(1)) / (float)(sample_size-1)));
    result.push_back(p_temp);
  }
  result.push_back(p2_);

  return result;
}

//判断line是否可见　0:不可见 1:局部可见 2:（基本）完全可见
int isVisible(const vector<Point2i_>& samples, int* pixelLabelBuild, int* pixelLabelFace, int b, int f)
{
  //TOOD check
  int size = samples.size();
  int count = size;
  for(int i=0;i<size;i++)
  {
    int u,v;
    // cout<<u<<"  "<<v<<endl;
    u = samples[i](0);
    v = samples[i](1);
    if(u<=2 || u>= kWidth-3 || v<=2 || v>=kHeight-3){
      count--;
      continue;
    }
    bool isVis = false;
    /*
      判断当前采样点是否可见。
      方法１:简单临域判别
      方法２:描述子汉明距离匹配加速
    */
    //方法１
    for(int idx=0;idx<patternNum;idx++)
    {
      int dx = patternP[idx][0];
      int dy = patternP[idx][1];
      if(pixelLabelBuild[u+dx+(v+dy)*kWidth] != b){
        continue;
      }
      else if(pixelLabelFace[u+dx+(v+dy)*kWidth] !=f){
        continue;
      }
      else{
        isVis =true;
        break;
      }
    }
    //方法２ TODO

    if(isVis)
    {
      continue;
    }
    else
      count--;
  }

  if (count==size || count==size-1 || count == size-2){ //TODO
    return 2;
  }
  else if (count>0){
    return 1;
  }
  else{
    return 0;
  }
}

//判断line point 是否可见
vector<bool> isVisible_points(const vector<Point2i_>& samples, int* pixelLabelBuild, int* pixelLabelFace, int b, int f)
{
  int size = samples.size();
  vector<bool> result(size);
  for(int i=0;i<size;i++)
  {
    int u,v;
    // cout<<u<<"  "<<v<<endl;
    u = samples[i](0);
    v = samples[i](1);
    if(u<=2 || u>= kWidth-3 || v<=2 || v>=kHeight-3){
      continue;
    }
    bool isVis = false;
    /*
      判断当前采样点是否可见。
      方法１:简单临域判别
      方法２:描述子汉明距离匹配加速
    */
    //方法１
    for(int idx=0;idx<patternNum;idx++)
    {
      int dx = patternP[idx][0];
      int dy = patternP[idx][1];
      if(pixelLabelBuild[u+dx+(v+dy)*kWidth] != b){
        continue;
      }
      else if(pixelLabelFace[u+dx+(v+dy)*kWidth] !=f){
        continue;
      }
      else{
        isVis =true;
        break;
      }
    }
    result[i] = isVis;
  }
  return result;

}

//TODO
// void plotVisiblePoint(vector<Polygon> visibleFaces, BuildingSet masterSet)
// {
//   for (size_t i=0;i<visibleFaces.size();i++)
//   {
//     Polygon visFace = visibleFaces[i];
//     int buildlabel = visFace.getbuildlabel();
//     Building buildnow = masterSet.getBuilding(buildlabel);
//     Color cl = buildnow.getcolor();
//
//   }
// }

//read query image's corresponding build bounding boxs.
vector<Point4i_> read_bbx(const string& bbx_src)
{
  vector<Point4i_> result;
  ifstream infile(bbx_src);
  if(!infile.is_open())
  {
      cout<<"ERROR: "<<bbx_src<<" doesn't exist."<<endl;
      exit(1);
  }
  string line;
  while(getline(infile,line))
  {
    stringstream ss(line);
    string temp;
    vector<int> pt;
    while(ss>>temp)
    {
      pt.push_back(stoi(temp));
    }
    if (pt.size()!=4)
    {
      cout<<"ERROR: bbx form seems to be wrong, please check txt file"<<endl;
      exit(1);
    }
    Point4i_ ptt;
    for(size_t i=0;i<4;i++)
    {
      ptt[i] = pt[i];
    }
    result.push_back(ptt);
  }
  return result;
}

float computeIOU(Point4i_ bbx, vector<Point4i_> build_iou)
{
  int size = build_iou.size();
  int count = 0;
  for(size_t i=0;i<build_iou.size();i++)
  {
    int px,py;
    px = build_iou[i][0];
    py = build_iou[i][1];
    if(px<bbx[0]+bbx[2]+10 && px>bbx[0]-10 &&
       py<bbx[1]+bbx[3]+10 && py>bbx[1]-10){
       count++;
    }
  }
  return (float)count / (float)size;
}

float grasp_once(const camera_related& camera,
                 BuildingSet& masterSet,
                 const cv::Mat& query,
                 const vector<Point4i_>& building_bbx)
{
  /*
    <<<<<<<<<<<<<<<<<<<< FIRST PERIOD >>>>>>>>>>>>>>>>>>>>.
    :extract visible face and line from virtual building models.
  */
  //time period counter
  clock_t start_time, end_time;
  start_time = clock();

  cout<<"<<<stage 1>>>"<<endl;
  static int loop_n = -1;
  loop_n++;
  vector<Polygon> visibleFaces; //local variable
  //set output file
  std::ostringstream headerStream;
  string outfile = "out"+std::to_string(loop_n);
#if WRITE_PFM
  headerStream <<"PF\n";
  headerStream <<kWidth <<' '<<kHeight<<'\n';
  headerStream <<"-1.0\n"
  std::ofstream fileStream(outfile+".pfm",std::ios::out | std::ios::binary);
  // std::ofstream pointStream("out_point.pfm",std::ios::out | std::ios::binary);
#else
  headerStream <<"P6\n";
  headerStream <<kWidth <<' '<<kHeight <<'\n';
  headerStream <<" 255\n";
  std::ofstream fileStream(outfile+".ppm",std::ios::out | std::ios::binary);
  // std::ofstream pointStream("out_point.pfm",std::ios::out | std::ios::binary);
#endif
  fileStream << headerStream.str();
  // pointStream << headerStream.str();
  cout<<"masterSet.size: "<<masterSet.size()<<endl;
  masterSet.clearTree();
  masterSet.buildTree(camera.m_origin); //TODO
  std::vector<std::vector<int>> build_face(masterSet.size());
  cout<<"start projection"<<endl;
  for (size_t y=0; y<kHeight;y++)
  {
    //ppm are top-down
    // float yu = 1.0 - (float(y) / float(kHeight -1 ));
    float yu = float(y) / float(kHeight -1);
    for (size_t x=0;x<kWidth;x++)
    {
      float xu = float(x) / float(kWidth -1);
      cout<<"project at:<" <<xu<<", "<<yu<<">"<<endl;
      //find where this pixel hit in the scene
      Ray ray = makeCameraRay(camera.m_fieldOfViewInDegrees1,
                              camera.m_fieldOfViewInDegrees2,
                              camera.m_origin,
                              camera.m_target,
                              camera.m_targetUpDirection,
                              xu,yu);
      Intersection intersection(ray);
      masterSet.pickBuilding(intersection);//TODO
      bool intersected = masterSet.intersect(intersection);

      if (intersection.m_intersectedFace_label[0] >= 0 &&
          intersection.m_intersectedFace_label[1] >=0)
      {
        int bu = intersection.m_intersectedFace_label[0];
        int fa = intersection.m_intersectedFace_label[1];
        pixelLabelBuild[y*kWidth+x] = bu;
        pixelLabelFace[y*kWidth+x] = fa;

        //if not find label[1]
        std::vector<int> temp_f = build_face[bu];
        std::vector<int>::iterator iter = std::find(temp_f.begin(),temp_f.end(),fa);
        if(iter==temp_f.end()) //didn't find same value
        {
          build_face[bu].push_back(fa);
          Building buildtemp;
          Polygon temp;
          buildtemp = masterSet.getBuilding(bu);
          temp = buildtemp.getPolygon(fa);
          visibleFaces.push_back(temp);
        }
      }
      else //label -1 for non-intersection pixel.
      {
        pixelLabelBuild[y*kWidth+x] = -1;
        pixelLabelFace[y*kWidth+x] = -1;
      }


      Color pixelColor(0.0,0.0,0.0);
      if(intersected)
      {
        pixelColor = intersection.m_color;
      }
  #if WRITE_PFM
      fileStream <<pixelColor.m_r << pixelColor.m_g<<pixelColor.m_b;
  #else
      pixelColor.clamp();
      unsigned char r,g,b;
      r = static_cast<unsigned char>(pixelColor.m_r * 255.0);
      g = static_cast<unsigned char>(pixelColor.m_g * 255.0);
      b = static_cast<unsigned char>(pixelColor.m_b * 255.0);
      fileStream << r <<g<<b;
  #endif
    }
  }
  cout<<"visible face size: "<<visibleFaces.size()<<endl;
  fileStream.flush();
  fileStream.close();

  // pointStream.flush();
  // pointStream.close();

  //time count period end
  end_time = clock();
  double duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  cout<<"face extraction finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;
  cout<<"start line extracting process......"<<endl;
  start_time = clock();
  /*
    line extraction process,
    提取可视面特征上的所有可见线段特征：
    1.构建一个hashmap储存所有像素点-key 对应的面label信息-value
    2.遍历全体可视面：
      对每一个面的所有边，判断边的可见性0,1,2
    3.打印结果
  */
  //1. done  pixelLabelBuild / pixelLabelFace
  //２. 3.遍历全体可视面，并打印结果
  unordered_map<int,int> points_map;
  int count = 0;
  //ptx, pty, build_label,  edge_label(0:normal edge, 1:bottom edge, 2:top edge)
  vector<vector<Point4i_>> build_point;
  // vector<Point4i_> allPoints;
  cv::Mat point_img = cv::Mat::zeros(cv::Size(kWidth, kHeight), CV_8UC3);
  point_img.setTo(0);
  for(size_t i=0; i<visibleFaces.size();i++)
  {
    Polygon& face_now = visibleFaces[i];
    int face_label_now = face_now.getfacelabel();
    int build_label_now = face_now.getbuildlabel();
    // cout<<"build_label_now:"<<build_label_now<<endl;
    // cout<<face_now.getnormal()<<endl;
    // cout<<face_now.m_faceVisibility<<endl;
    for (size_t j=0;j<face_now.getlines().size();j++)
    {
      Line line_now = face_now.getline(j);
      /*
        离散化:　对当前直线按照比例离散成一定数量采样点
      */
      Pointf_ p1, p2;
      p1 = line_now.m_p1;
      p2 = line_now.m_p2;
      // cout<<"endpoint: ["<<p1<< " ] -> [ "  <<p2<<" ] ****** "
      //     <<"build->face: "<<build_label_now<<" -> "<<face_label_now<<endl;
      vector<Point2i_> tt = do_sample(p1, p2, camera);
      line_now.m_visibility = isVisible(tt,
                                        pixelLabelBuild,
                                        pixelLabelFace,
                                        build_label_now,
                                        face_label_now);
      face_now.setLineVisibility(j, line_now.m_visibility);
      if (line_now.m_visibility > 0)   //局部可见或完全可见
      {
        std::ofstream outTxt("visible_info_6d_estimate.txt",ios_base::app);
        outTxt.setf(ios_base::fixed);
        outTxt.precision(6);

        cout<<"current line is part or full visible: {　"<<build_label_now<<" -> "
            <<face_label_now<<" -> "<<line_now.m_label<<"　}"
            <<"  visibility: "<<line_now.m_visibility<<" (1: patial visible; 2: full visible)"
            <<endl;

        outTxt<<"current line is part or full visible: {　"<<build_label_now<<" -> "
               <<face_label_now<<" -> "<<line_now.m_label<<"　}"
               <<"  visibility: "<<line_now.m_visibility<<" (1: patial visible; 2: full visible)"
               <<endl;
        outTxt.close();
      }

      vector<bool> tt_visibility = isVisible_points(tt,
                                        pixelLabelBuild,
                                        pixelLabelFace,
                                        build_label_now,
                                        face_label_now);
      for(size_t k=0;k<tt_visibility.size();k++)
      {
        if(tt_visibility[k]==true)
        {
          //plot tt[k];

          // clt.setcolor(build_label_now);
          Building build_now_ = masterSet.getBuilding(build_label_now);
          Color clt = build_now_.getcolor();
          //     Color cl = buildnow.getcolor();
          cv::Point p(tt[k](0),tt[k](1));
          cv::circle(point_img, p, 3, cv::Scalar(int(clt.m_r*255.0),int(clt.m_g*255.0), int(clt.m_b*255.0)), -1);  // 画半径为1的圆(画点）

          Point4i_ ptt;
          ptt[0] = tt[k](0);
          ptt[1] = tt[k](1);
          ptt[2] = build_label_now;
          if(face_label_now == build_now_.getPolygons().size()-2)
          {
            ptt[3] = 2;
          }
          else if(face_label_now == build_now_.getPolygons().size()-1)
          {
            ptt[3] = 1;
          }
          else{
            if(j==0) ptt[3] =2;
            else if(j==2) ptt[3] =1;
            else ptt[3] =0;
          }

          unordered_map<int,int>::const_iterator got = points_map.find(build_label_now);
          if(got == points_map.end())
          {
            build_point.push_back(vector<Point4i_>());
            // build_point[count].push_back(ptt);
            pair<int,int> temp;
            temp.first = build_label_now;
            temp.second = count++;
            points_map.insert(temp);
            build_point[build_point.size()-1].push_back(ptt);
          }
          else
          {
            build_point[got->second].push_back(ptt);
          }
          // allPoints.push_back(ptt);  //double-time of actual size
        }
      }
    }
  }

  // vector<Pointi_> Points_builds;
  // while(!allPoints.isEmpty())
  // {
  //   Point4i_ = allPoints.back();
  // }
  // cv::imshow("visible points", point_img);
  cv::imwrite(outfile+".jpg", point_img);
  // plotVisiblePoint(visibleFaces);
  //detele dynamic objects.
  end_time = clock();
  duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  cout<<"line extraction finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;

  /*
    <<<<<<<<<<<<<<<<<<<< SECOND PERIOD >>>>>>>>>>>>>>>>>>>>.
    :projecting and scoring. to obtain best matching between query image pose
    and virtual model pose.
    1.对当前匹配虚拟帧，通过图像建筑boundary box判断交并比是否合理
    2.对满足1条件的匹配建立图像金字塔进行像素梯度加权打分.
  */
  start_time = clock();
  //find a score
  float score = -1.0;
  //1.建筑轮廓boundary box判断当前匹配是否合理，若不合理则舍弃当前匹配; 若合理则进行梯度金字塔打分
  for(size_t i=0;i<building_bbx.size();i++)
  {
    Point4i_ bbx = building_bbx[i];
    float iou = 0.0;
    for(size_t j=0;j<build_point.size();j++)
    {
      vector<Point4i_> build_iou = build_point[j];
      float iou_temp = computeIOU(bbx,build_iou);
      if(iou_temp>iou){
        iou = iou_temp;
      }
      cout<<"iou: "<<iou<<endl;
    }
    if(iou<iou_threhold)
    {
      cout<<"ERROR"<<endl;
      return score; //-1
    }
  }

  //2.图像金字塔求像素梯度打分
  float score_pyr[PYR_LEVELS]={0.0};
  //建立金字塔once
  if(wG[0]==0){
    setCalib(kWidth, kHeight, camera.K);
  }
  //输入金字塔图像
  FrameShell FrameQuery;
  FrameQuery.makeImages(query);
  //建立可视点到金字塔打分
  for (int lvl=0; lvl<pyrLevelsUsed; lvl++){
    for(int i=0;i<build_point.size();i++){
      vector<Point4i_> build_pt = build_point[i];
      for(int j=0;j<build_pt.size();j++)
      {
        Point4i_ pt = build_pt[j];
        int px_pyr = int(pt[0] / pow(2,lvl));
        int py_pyr = int(pt[1] / pow(2,lvl));
        if(px_pyr>wG[lvl]-1) px_pyr = wG[lvl]-1;
        if(py_pyr>hG[lvl]-1) py_pyr = hG[lvl]-1;
        if(pt[3] == 2) //top edge point, pass.
        continue;
        score_pyr[lvl] += FrameQuery.absSquaredGrad[lvl][py_pyr*wG[lvl]+px_pyr];
      }
    }
    score += weights[lvl] * score_pyr[lvl];
  }
  cout<<"current score: "<<score<<endl;
  std::ofstream scoreOut("estimate_pose_scores.txt",ios_base::app);
  scoreOut.setf(ios_base::fixed);
  scoreOut.precision(6);
  scoreOut<<"current estimate pose has score of: { "<< score <<" }"<<endl;
  scoreOut.close();

  end_time = clock();
  duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  cout<<"projection finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;

  return score;
}


int main(int argc, char** argv)
{
  //(1)-loading parameter.
  /*
      This raw camera pose should come from raw pose estimation from geo-CNN
      or basically use GPS and compass from ceilphone,
      which will be set to the search start pose.
  */
  double lon;  //raw pose-long
  double lat;  //raw pose-lat
  float yaw;      //raw pose-yaw
  string image_src, bbx_src; //query and corresponding builds bounding box.
  int loop_x;//TODO
  int loop_y;//TODO
  int loop_t;//TODO
  // int loop_f1;
  // int loop_f2;

  float gap_x; //X search range;
  float gap_y; //Y search range;
  float gap_theta; //yaw angle search range;
  // float gap_f1;
  // float gap_f2;

  bpo::options_description desc("Program options");
  desc.add_options()
    //Options
    ("image_src",bpo::value<string>(&image_src)
                  -> default_value("/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/querys/009900_1.jpg"),
                  "input image file path, Usage: --image_src XXX.xxx")
    ("bbx_src",bpo::value<string>(&bbx_src)
                                -> default_value("/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/querys/009900_1_bbx.txt"),
                                "input image's bbx file path, Usage: --bbx_src XXX.txt")
    ("start_lon", bpo::value<double>(&lon) -> default_value(-74.008772),
                  "raw pose long, Usage: --start_lon xxx")
    ("start_lat",bpo::value<double>(&lat) -> default_value(40.743051),
                  "raw pose latitude, Usage: --start_lat xxx")
    ("start_yaw",bpo::value<float>(&yaw) -> default_value(56.89),
                  "raw pose yaw, Usage: --start_yaw xxx")
    ("loop_x",bpo::value<int>(&loop_x) -> default_value(1),
                  "search step/2 in x, Usage: --loop_x xxx")
    ("loop_y",bpo::value<int>(&loop_y) -> default_value(1),
                  "search step/2 in y, Usage: --loop_y xxx")
    ("loop_t",bpo::value<int>(&loop_t) -> default_value(2),
                  "search step/2 in yaw angle, Usage: --loop_x xxx")
    // ("loop_f1",bpo::value<int>(&loop_f1) -> default_value(1),
    //               "search step/2 in fov1 angle, Usage: --loop_f1 xxx")
    // ("loop_f2",bpo::value<int>(&loop_f2) -> default_value(1),
    //               "search step/2 in fov2 angle, Usage: --loop_f2 xxx")
    ("gap_x",bpo::value<float>(&gap_x) -> default_value(10.0),
                  "search range in x, Usage: --gap_x xxx")
    ("gap_y",bpo::value<float>(&gap_y) -> default_value(10.0),
                  "search range in y, Usage: --gap_y xxx")
    ("gap_t",bpo::value<float>(&gap_theta) -> default_value(30.0),
                  "search range in yaw angle, Usage: --gap_theta xxx");
    // ("gap_f1",bpo::value<float>(&gap_f1) -> default_value(10.0),
    //               "search range in fov1 angle, Usage: --gap_f1 xxx")
    // ("gap_f2",bpo::value<float>(&gap_f2) -> default_value(10.0),
    //               "search range in fov2 angle, Usage: --gap_f2 xxx");
  // Parse the command line
  bpo::variables_map vm;
  bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
  // option help
  if (vm.count("help"))
  {
    cout << desc << "\n";
    return false;
  }
  // Process options.
  bpo::notify(vm);

  Point2d_ camera_origin_d(lon, lat);  //long_lat
  lonlatToXY(camera_origin_d, origin_lon_lat);  //change to X,Y
  Pointf_ camera_origin_raw((float)camera_origin_d(0),(float)camera_origin_d(1),40.0);
  float theta = yaw * M_PI / 180.0;  //TODO
  // Eigen::Vector3f target(camera_origin_raw(0)+10.0*sin(theta),camera_origin_raw(1)+10.0*cos(theta),camera_origin_raw(2)+1.5);  //TODO
  // Eigen::Vector3f targetUpDirection(0.0,0.0,1.0);
  cv::Mat query_img = cv::imread(image_src,  0);  // 0: gray / 1: rgb
  vector<Point4i_>building_bbx;
  building_bbx = read_bbx(bbx_src);

  cv::Mat bbx_img;
  bbx_img = query_img.clone();
  for(size_t i=0;i<building_bbx.size();i++)
  {
    Point4i_ bbx_temp = building_bbx[i];
    cv::Rect rect(bbx_temp[0], bbx_temp[1], bbx_temp[2], bbx_temp[3]);//左上坐标（x,y）和矩形的长(x)宽(y)
    cv::rectangle(bbx_img, rect, Scalar(255, 0, 0),1, LINE_8,0);
  }
  cv::imshow("with bbx",bbx_img);
  cv::waitKey(0); //check the bbx

  //(2)-set scene
  BuildingSet masterSet;
  BuildingReader DataReader(inputdir, origin_lon_lat, camera_origin_raw);//choose building in a range
  Building* temps[DataReader.m_buildings.size()];
  for (size_t i=0;i<DataReader.m_buildings.size();i++)
  {
    //TODO　how to delete?
    temps[i] = new Building(DataReader.m_buildings[i].getPoints(),DataReader.m_buildings[i].getUp(),
    DataReader.m_buildings[i].getDown());
    masterSet.addBuilding(temps[i]);
    // delete temp;
  }
  cout<<"masterSet construted."<<endl;

  /*(3)
    Searching loop start here:
    1.use simple loop method, or;
    2.use non-lineal search such as g2o
  */
  //method 1: simple loop...
  float score = -1.0;
  int ans[3];
  float fov[2];  //TODO
  float dz, dp;  //TODO
  for(int s_x=-(loop_x-1);s_x<loop_x;s_x++)
  {
    for(int s_y=-(loop_y-1);s_y<loop_y;s_y++)
    {
      for(int s_t=-(loop_t-1);s_t<loop_t;s_t++)
      {
        // for(int s_f1 = -(loop_f1-1);s_f1<loop_f1;s_f1++)
        // {
        //  for (int s_f2 = -(loop_f2-1);s_f2<loop_f2;s_f2++)
        // {
        //   for(int s_z = -9;s_z<10;s_z++)
        //   {
        //    for (int s_p = -9;s_p<10;s_p++)
        //   {
        float DX = loop_x !=1 ? s_x * (gap_x/(2.0*(loop_x-1))) : 0;
        float DY = loop_y !=1 ? s_y * (gap_y/(2.0*(loop_y-1))) : 0;
        float DT = loop_t !=1 ? s_t * (gap_theta/(2.0*(loop_t-1))) : 0;
        //
        // float DF1 = loop_f1 !=1 ? s_f1 * (gap_f1/(2.0*(loop_f1-1))) : 0;
        // float DF2 = loop_f2 !=1 ? s_f2 * (gap_f2/(2.0*(loop_f2-1))) : 0;
        // float DZ = s_z * (30.0 /(2.0*9));
        // float DP = s_p * (10.0 /(2.0*9));

        Pointf_ camera_origin_current((float)camera_origin_d(0)+DX,(float)camera_origin_d(1)+DY,40.0);
        float theta_current = (yaw+DT) * M_PI / 180.0;  //TODO
        Eigen::Vector3f target_current(camera_origin_current(0)+10.0*sin(theta_current),
                               camera_origin_current(1)+10.0*cos(theta_current),
                               camera_origin_current(2)+1.5);  //TODO
        Eigen::Vector3f targetUpDirection_current(0.0,0.0,1.0);
        camera_related camera_current(82.0,
                                      130.0,
                                      camera_origin_current,
                                      target_current,
                                      targetUpDirection_current);
        float score_temp = grasp_once(camera_current, masterSet, query_img, building_bbx);
        if(score_temp > score)
        {
          score = score_temp;
          ans[0] = s_x;
          ans[1] = s_y;
          ans[2] = s_t;
          // fov[0] = 82.0+DF1;
          // fov[1] = 130.0+DF2;
          // dz = 40+DZ;
          // dp = 1.5+DP;
        }
      }
      //    }
      //    }
      // }
      // }
    }
  }

  std::ofstream adjust("adjust.txt",ios_base::app);
  adjust.setf(ios_base::fixed);
  adjust.precision(6);
  adjust<<fov[0]<<"  "<<fov[1]<<"  "
        <<dz<<"  "<<dp<<endl;
  adjust.close();

  // cout<<"best fov angle: "<<fov[0]<<" "<<fov[1]<<endl;
  cout<<"best shoot of estimated pose comes from: ";
  for(int i=0;i<3;i++)
  {
    cout<<" "<<ans[i];
  }
  cout<<endl<<"with score of: "<<score<<endl;
  cout<<" done!"<<endl;
  int fignum = (ans[0]+loop_x-1)*(2*loop_y-1)*(2*loop_t-1)+
               (ans[1]+loop_y-1)*(2*loop_t-1)+
               (ans[2]+loop_t-1);
  string bestfile = "out"+std::to_string(fignum)+".jpg";
  cv::Mat best_est = cv::imread(bestfile);
  cv::imshow("best shoot", best_est);
  cv::waitKey(0);

  // for (size_t i=0;i<DataReader.m_buildings.size();i++)
  // {
  //   delete temps[i];
  // }

  /*
    <<<<<<<<<<<<<<<<<<<< SECOND PERIOD >>>>>>>>>>>>>>>>>>>>.
    :projecting and scoring. to obtain best matching between query image pose
    and virtual model pose.
  */
  // start_time = clock();
  /*
    1.对一张query图像，按照其GT，人为设定一个粗糙搜索范围(模拟有误差的GPS和指南针)
    ２．遍历粗糙搜索范围，二分法或梯度下降寻找像素平均梯度打分最高或特征点平均描述子打分最高的结果.
  */


  // end_time = clock();
  // duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  // cout<<"projection finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;

  return 0;
}
