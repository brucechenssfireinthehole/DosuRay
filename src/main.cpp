/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#include "DosuRay.h"
#include "DataReader.h"

using namespace DosuRay;
// using namespace std;

vector<Polygon> visibleFaces;
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


int main(int argc, char** argv)
{
  /*
  //construct inputfile.txt from .json file
  // ifstream is("/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/rows.json",ios::binary);
  // if(!is.is_open()){
  //   cout<<"ERROR: open json file failed."<<endl;
  //   return -1;
  // }
  // cout<<"Data load finished."<<endl;
  // string outfile = "/home/dosu/personal/code/beta/study_code/geo_localization/DosuRay/data/pickup.txt";
  // ofstream outFile(outfile,ios_base::ate|ios_base::out);
  // // outFile.setf(ios_base::fixed);
  // // outFile.precision(6);
  //
  // Json::Reader reader;
  // Json::Value root;
  //
  // assert(reader.parse(is,root));
  // Json::Value data;
  // data = root["data"];
  // cout<<"Data parse finished."<<endl;
  // for (int index=0;index<data.size();index++)
  // {
  //   int bin = std::stoi(data[index][11].asString());
  //   string footprints = data[index][8].asString();
  //   vector<Point2d> ss = strToPoint2d(footprints); //long-lat need convert to float
  //   double groundelev = std::stod(data[index][16].asString());
  //   double heightroof = std::stod(data[index][14].asString());
  //   outFile<<"build:"<<bin<<" X:";
  //   for (size_t i=0;i<ss.size();i++)
  //   {
  //     outFile<<ss[i](0)<<" ";
  //   }
  //   outFile<<"Y:";
  //   for (size_t i=0;i<ss.size();i++)
  //   {
  //     outFile<<ss[i](1)<<" ";
  //   }
  //   outFile<<"up:"<<heightroof+groundelev<<" down:"<<groundelev<<endl;
  //
  // }
  // outFile.close();
  // return 1;
  */
  /*
    <<<<<<<<<<<<<<<<<<<< FIRST PERIOD >>>>>>>>>>>>>>>>>>>>.
    :extract visible face and line from virtual building models.
  */
  //time period counter
  clock_t start_time, end_time;
  start_time = clock();
  //set camera
  Point2d_ camera_origin_d(-74.008772,40.743051);  //long_lat
  lonlatToXY(camera_origin_d, origin_lon_lat);
  Pointf_ camera_origin((float)camera_origin_d(0),(float)camera_origin_d(1),40.0);
  float theta = (56.89) * M_PI / 180.0;  //TODO
  Eigen::Vector3f target(camera_origin(0)+10.0*sin(theta),camera_origin(1)+10.0*cos(theta),camera_origin(2)+1.5);  //TODO
  Eigen::Vector3f targetUpDirection(0.0,0.0,1.0);
  camera_related camera(82.0,
                        130.0,
                        camera_origin,
                        target,
                        targetUpDirection);

  //set scene
  BuildingSet masterSet;
  //put a simple ground Plane
  // Plane plane(Pointf_(0.0,-2.0,0.0), Eigen::Vector3f(0.0,1.0,0.0), Color(0.6,0.5,0.8));
  // masterSet.addShape(&plane);
  //TODO
  //load data
  /*
  //build1
  vector<Point2f__> points_raw1;
  Point2f__ p1(0.0,0.0);
  Point2f__ p2(10.0,-3.0);
  Point2f__ p3(15.0,20.0);
  Point2f__ p4(5.0,26.0);
  points_raw1.push_back(p1);
  points_raw1.push_back(p2);
  points_raw1.push_back(p3);
  points_raw1.push_back(p4);
  Building build1(points_raw1, 25.0, 5.0);  //TODO
  masterSet.addBuilding(&build1);


  // build2
  vector<Point2f__> points_raw2;
  Point2f__ p1_2(15.0,20.0);
  Point2f__ p2_2(27.0,30.0);
  Point2f__ p3_2(30.0,40.0);
  Point2f__ p4_2(17.0,45.0);
  Point2f__ p5_2(10.0,26.0);
  points_raw2.push_back(p1_2);
  points_raw2.push_back(p2_2);
  points_raw2.push_back(p3_2);
  points_raw2.push_back(p4_2);
  points_raw2.push_back(p5_2);
  Building build2(points_raw2, 30.0, 7.0);  //TODO
  masterSet.addBuilding(&build2);

  // build3
  vector<Point2f__> points_raw3;
  Point2f__ p1_3(23.0,15.0);
  Point2f__ p2_3(50.0,30.0);
  Point2f__ p3_3(55.0,50.0);
  Point2f__ p4_3(40.0,51.0);
  Point2f__ p5_3(35.0,41.0);
  points_raw3.push_back(p1_3);
  points_raw3.push_back(p2_3);
  points_raw3.push_back(p3_3);
  points_raw3.push_back(p4_3);
  points_raw3.push_back(p5_3);
  Building build3(points_raw3, 26.0, 2.0);  //TODO
  masterSet.addBuilding(&build3);

  // build4
  vector<Point2f__> points_raw4;
  Point2f__ p1_4(-2.0,15.0);
  Point2f__ p2_4(-2.0,30.0);
  Point2f__ p3_4(-40.0,35.0);
  Point2f__ p4_4(-35.0,11.0);
  // Point2f__ p5_4(35.0,41.0);
  points_raw4.push_back(p1_4);
  points_raw4.push_back(p2_4);
  points_raw4.push_back(p3_4);
  points_raw4.push_back(p4_4);
  // points_raw3.push_back(p5_3);
  Building build4(points_raw4, 15.0, 2.0);  //TODO
  masterSet.addBuilding(&build4);
  */
  BuildingReader DataReader(inputdir, origin_lon_lat, camera_origin);
  cout<<"<<<stage 1>>>"<<endl;
  // return 1;
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

  //set output file
  std::ostringstream headerStream;
#if WRITE_PFM
  headerStream <<"PF\n";
  headerStream <<kWidth <<' '<<kHeight<<'\n';
  headerStream <<"-1.0\n"
  std::ofstream fileStream("out.pfm",std::ios::out | std::ios::binary);
  // std::ofstream pointStream("out_point.pfm",std::ios::out | std::ios::binary);
#else
  headerStream <<"P6\n";
  headerStream <<kWidth <<' '<<kHeight <<'\n';
  headerStream <<" 255\n";
  std::ofstream fileStream("out.ppm",std::ios::out | std::ios::binary);
  // std::ofstream pointStream("out_point.pfm",std::ios::out | std::ios::binary);
#endif
  fileStream << headerStream.str();
  // pointStream << headerStream.str();

  cout<<"masterSet.size: "<<masterSet.size()<<endl;
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

  cv::Mat point_img = cv::Mat::zeros(cv::Size(kWidth, kHeight), CV_8UC3);
  point_img.setTo(0);
  for(int i=0; i<visibleFaces.size();i++)
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
        std::ofstream outFile("visible_info.txt",ios_base::app);
        outFile.setf(ios_base::fixed);
        outFile.precision(6);

        cout<<"current line is part or full visible: {　"<<build_label_now<<" -> "
            <<face_label_now<<" -> "<<line_now.m_label<<"　}"
            <<"  visibility: "<<line_now.m_visibility<<" (1: patial visible; 2: full visible)"
            <<endl;

        outFile<<"current line is part or full visible: {　"<<build_label_now<<" -> "
               <<face_label_now<<" -> "<<line_now.m_label<<"　}"
               <<"  visibility: "<<line_now.m_visibility<<" (1: patial visible; 2: full visible)"
               <<endl;
        outFile.close();
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

        }
      }

    }
  }
  // cv::imshow("visible points", point_img);
  cv::imwrite("visible_points.jpg", point_img);
  // plotVisiblePoint(visibleFaces);
  //detele dynamic objects.
  for (size_t i=0;i<DataReader.m_buildings.size();i++)
  {
    delete temps[i];
  }
  end_time = clock();
  duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  cout<<"line extraction finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;

  // /*
  //   <<<<<<<<<<<<<<<<<<<< SECOND PERIOD >>>>>>>>>>>>>>>>>>>>.
  //   :projecting and scoring. to obtain best matching between query image pose
  //   and virtual model pose.
  // */
  // start_time = clock();
  // /*
  //   1.对一张query图像，按照其GT，人为设定一个粗糙搜索范围(模拟有误差的GPS和指南针)
  //   ２．遍历粗糙搜索范围，二分法或梯度下降寻找像素平均梯度打分最高或特征点平均描述子打分最高的结果.
  // */
  //
  //
  // end_time = clock();
  // duration = (double)(end_time - start_time) / CLOCK_PER_SEC;
  // cout<<"projection finished, with time period>>>>>>> "<< duration<<" s <<<<<<"<<endl;

  return 0;
}
