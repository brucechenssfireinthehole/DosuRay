/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef DOSURAY_H
#define DOSURAY_H

//common support: PCL/Eigen...
#include "Common.h"
//support Rbtree
#include "Rbtree.h"

using namespace std;

namespace DosuRay
{
  //define rgb Color class
  struct Color
  {
    float m_r, m_g, m_b;
    Color() : m_r(0.0), m_g(0.0), m_b(0.0){}
    Color(const Color& c) : m_r(c.m_r), m_g(c.m_g), m_b(c.m_b){}
    Color(float r, float g, float b) : m_r(r), m_g(g), m_b(b){}
    explicit Color(float f) : m_r(f), m_g(f), m_b(f){}

    void rand_normalcolor()
    {
    	int *rgb = new int[3];
    	rgb[0] = rand() % 255;
    	rgb[1] = rand() % 255;
    	rgb[2] = rand() % 255;
      m_r  = rgb[0] / 255.0;
      m_g =  rgb[1] / 255.0;
      m_b =  rgb[2] / 255.0;
    	// return rgb;
    }

    //TODO
    void setcolor(int num)
    {
      int *rgb = new int[3];
      rgb[0] = (num) % 255;
      rgb[1] = (num +100) % 255;
      rgb[2] = (num * num) % 255;
      m_r = rgb[0] / 255.0;
      m_g = rgb[1] / 255.0;
      m_b = rgb[2] / 255.0;
    }

    void clamp(float mi = 0.0, float ma = 1.0)
    {
      m_r = std::max(mi, std::min(ma,m_r));
      m_g = std::max(mi, std::min(ma,m_g));
      m_b = std::max(mi, std::min(ma,m_b));
    }

    Color& operator =(const Color& c)
    {
      m_r = c.m_r;
      m_g = c.m_g;
      m_b = c.m_b;
      return *this;
    }

    Color& operator +=(const Color& c)
    {
      m_r += c.m_r;
      m_g += c.m_g;
      m_b += c.m_b;
      return *this;
    }

    Color& operator -=(const Color& c)
    {
      m_r -= c.m_r;
      m_g -= c.m_g;
      m_b -= c.m_b;
      return *this;
    }

    Color& operator *=(const Color& c)
    {
      m_r *= c.m_r;
      m_g *= c.m_g;
      m_b *= c.m_b;
      return *this;
    }

    Color& operator /=(const Color& c)
    {
      m_r /= c.m_r;
      m_g /= c.m_g;
      m_b /= c.m_b;
      return *this;
    }

    Color& operator *=(float f)
    {
      m_r *= f;
      m_g *= f;
      m_b *= f;
      return *this;
    }

    Color& operator /=(float f)
    {
      m_r /= f;
      m_g /= f;
      m_b /= f;
      return *this;
    }
  };

  inline Color operator +(const Color& c1, const Color& c2)
  {
    return Color(c1.m_r+c2.m_r, c1.m_g+c2.m_g, c1.m_b+c2.m_b);
  }

  inline Color operator -(const Color& c1, const Color& c2)
  {
    return Color(c1.m_r-c2.m_r, c1.m_g-c2.m_g, c1.m_b-c2.m_b);
  }

  inline Color operator *(const Color& c1, const Color& c2)
  {
    return Color(c1.m_r*c2.m_r, c1.m_g*c2.m_g, c1.m_b*c2.m_b);
  }

  inline Color operator /(const Color& c1, const Color& c2)
  {
    return Color(c1.m_r / c2.m_r, c1.m_g / c2.m_g, c1.m_b / c2.m_b);
  }

  inline Color operator *(const Color& c1, float f)
  {
    return Color(c1.m_r*f, c1.m_g*f, c1.m_b*f);
  }

  inline Color operator /(const Color& c1, float f)
  {
    return Color(c1.m_r / f, c1.m_g / f, c1.m_b / f);
  }

  //define Ray class
  const float kRayTMin = 0.00001f;
  const float kRayTMax =1.0e30f;

  struct Ray
  {
    Pointf_ m_origin;
    Eigen::Vector3f m_direction;
    float m_tMax;
    float m_angle;
    //default
    Ray() : m_origin(), m_direction(0.0f,0.0f,0.0f), m_tMax(kRayTMax), m_angle(){}
    //copy
    Ray(const Ray& r): m_origin(r.m_origin), m_direction(r.m_direction), m_tMax(r.m_tMax), m_angle(r.m_angle){}
    //normal assign
    Ray(const Pointf_& origin, const Eigen::Vector3f& direction, float tMax = kRayTMax, float angle = -1.0)
      : m_origin(origin), m_direction(direction), m_tMax(tMax), m_angle(angle){}

    //operator
    Ray& operator =(const Ray& r)
    {
      m_origin = r.m_origin;
      m_direction = r.m_direction;
      m_tMax = r.m_tMax;
      m_angle  = r.m_angle;
      return *this;
    }
    Pointf_ calculate(float f) const
    {
      return (m_origin + f * m_direction);
    }
  };

  //define Intersection class
  //declare class Shape
  class Shape;

  struct Intersection
  {
    Ray m_ray;
    float m_t;
    int m_intersectedFace_label[2];
    Shape* m_pShape;
    Color m_color;
    Eigen::Vector3f m_normal;
    //default
    Intersection() : m_ray(), m_t(kRayTMax), m_pShape(NULL),
                     m_color(), m_normal(),m_intersectedFace_label{-1,-1}{}
    //copy
    Intersection(const Intersection& i) : m_ray(i.m_ray),
                                          m_t(i.m_t),
                                          m_pShape(i.m_pShape),
                                          m_color(i.m_color),
                                          m_normal(i.m_normal),
      m_intersectedFace_label{i.m_intersectedFace_label[0],i.m_intersectedFace_label[1]}{}

    Intersection(const Ray& r) : m_ray(r),
                                 m_t(r.m_tMax),
                                 m_pShape(NULL),
                                 m_color(),
                                 m_normal(),
                                 m_intersectedFace_label{-1,-1}{}
    //operator
    Intersection& operator =(const Intersection& i)
    {
      m_ray = i.m_ray;
      m_t = i.m_t;
      m_pShape = i.m_pShape;
      m_color = i.m_color;
      m_normal = i.m_normal;
      m_intersectedFace_label[0] = i.m_intersectedFace_label[0];
      m_intersectedFace_label[1] = i.m_intersectedFace_label[1];
      return *this;
    }

    bool intersected() const
    {
      return (m_pShape == NULL) ? false : true;
    }

    Pointf_ position() const
    {
      return m_ray.calculate(m_t);
    }
  };

  //define Shape base class
  // class Polygon;

  class Shape
  {
  public:
    virtual ~Shape(){}
    //derived class must define intersect()
    virtual bool intersect(Intersection& intersection) = 0;
    virtual int getlabel(){}
    virtual vector<Point2f_> getboundingbox(){}
    virtual Point2f_ getboxcenter(){}
    // virtual Polygon getPolygon(int num){}
    virtual int* calculate_bounding_range(const Point2f_& origin){}
    virtual void setlabel(int label){}
  };


  class Plane : public Shape
  {
  public:
    Plane(const Pointf_& position, Eigen::Vector3f normal, const Color& color)
            : m_position(position),
           // m_normal(normal.normalize())  //normalized
              m_color(color)
    {
      normal.normalize();
      m_normal = normal;
    }

    virtual ~Plane(){}
    virtual bool intersect(Intersection& intersection)
    {
      //Plane eq: ax+by+cz+d=0 or dot(n,p-p0)=0, in which n is m_normal, p0 is m_postion
      //and p is from single ray: p = ray_origin + t * ray_direction
      float nDotD = m_normal.dot(intersection.m_ray.m_direction);
      if(nDotD >0.0)
      {
        return false;
      }

      float t =(m_position.dot(m_normal)-intersection.m_ray.m_origin.dot(m_normal))/intersection.m_ray.m_direction.dot(m_normal);
      //choose closest t
      if(t>=intersection.m_t || t<kRayTMin)
      {
        return false;
      }
      intersection.m_t = t;
      intersection.m_pShape = this;
      intersection.m_color = m_color;
      intersection.m_normal = m_normal;

      return true;
    }

  protected:
    Pointf_ m_position;
    Eigen::Vector3f m_normal;
    Color m_color;
  };

//TODO ->　date 2019_08_01
  class Line
  {
  public:
    Line() :  m_p1(), m_p2(), m_color(),m_label(-1),
              m_label_face(-1),
              m_label_build(-1),
              m_visibility(-1){}
    Line(const Pointf_& p1, const Pointf_& p2, const Color& c, int label,
          int label_face)
          : m_p1(p1),
            m_p2(p2),
            m_color(c),
            m_label(label), m_label_face(label_face), m_label_build(-1),
            m_visibility(-1)
            {}
    void setbuildlabel(int label)
    {
        m_label_build = label;
    }

    Line& operator =(const Line& c)
    {
      m_p1 = c.m_p1;
      m_p2 = c.m_p2;
      m_color = c.m_color;
      m_label = c.m_label;
      m_label_face = c.m_label_face;
      m_label_build = c.m_label_build;
      m_visibility = c.m_visibility;
      return *this;
    }

    Pointf_ m_p1, m_p2;
    Color m_color;
    int m_label;  //line label
    int m_label_face; //face label for current line
    int m_label_build; //build label for current line
    int m_visibility; //0: 不可见　1:局部可见 2:完全可见
  };

  class Polygon :  public Shape
  {
  public:
    Polygon(){}
    Polygon(const vector<Pointf_>& PointList, const Eigen::Vector3f& normal, Color color, int label): m_build_label(-1)
    {
      vector<Pointf_> temp = PointList;
      temp.push_back(PointList[0]);
      for (size_t i=0;i<temp.size()-1;i++)
      {
        Line line(temp[i],temp[i+1],color,m_lines.size(),label);
        m_lines.push_back(line);
      }
      m_points = PointList;
      m_color = color;
      Eigen::Vector3f norm_tp = normal;
      norm_tp.normalize();
      m_normal = norm_tp;
      m_label = label;
      m_faceVisibility = false;  //default value
      // m_build_label = build_label;
    }

    virtual ~Polygon(){}

    Polygon& operator =(const Polygon& p)
    {
      m_points = p.m_points;
      m_color = p.m_color;
      m_normal = p.m_normal;
      m_lines = p.m_lines;
      m_label = p.m_label;
      m_build_label = p.m_build_label;
      m_faceVisibility = p.m_faceVisibility;
      return *this;
    }
    virtual bool intersect(Intersection &intersection)
    {
      Pointf_ position = m_points[0];
      float t =(position.dot(m_normal)-intersection.m_ray.m_origin.dot(m_normal))/intersection.m_ray.m_direction.dot(m_normal);
      //choose closest t
      if(t>=intersection.m_t || t<kRayTMin)
      {
        // cout<<"WARNNING1"<<endl;
        if (print_mode)
          cout<<"one face abodoned -> "<< m_label <<endl;
        return false;
      }

      Pointf_ tg = intersection.m_ray.m_origin + t * intersection.m_ray.m_direction;
      // cout<<tg<<endl;
      if (! inPolygon(tg))
      {
        // cout<<"WARNNING2"<<endl;
        if (print_mode)
          cout<<"one face abodoned -> "<< m_label <<endl;
        return false;
      }
      intersection.m_t = t;
      intersection.m_pShape = this;
      intersection.m_color = m_color;
      intersection.m_normal = m_normal;
      intersection.m_intersectedFace_label[0] = m_build_label;
      intersection.m_intersectedFace_label[1] = m_label;
      if (print_mode)
      cout<<"one face passed -> "<< m_label <<endl;

      return true;
    }

    Eigen::Matrix4f calculate_trans(const Eigen::Vector3f& normal_ss)
    {
      Eigen::Vector3f norm_temp = normal_ss;
    	Eigen::Matrix4f result =Eigen::Matrix4f::Identity();
    	float ca, cb, cc, norm;
    	ca = norm_temp(0);
    	cb = norm_temp(1);
    	cc = norm_temp(2);
    	norm = sqrt(ca*ca+cb*cb+cc*cc);
    	ca = ca / norm;
    	cb = cb / norm;
    	cc = cc / norm;
    	if (cc < 0)
    	{
    		ca = -ca;
    		cb = -cb;
    		cc = -cc;
    	}
    	// float temp = - cb / (ca *sqrt((ca*ca)/(ca*ca+cb*cb+EPS))+EPS);
    	// result(0,0) = temp > 0 ? temp : -1*temp;
    	// result(1,0) = - ca / (cb+EPS) *result(0,0);
    	// result(2,0) = 0.0;
    	// result(0,1) = ca;
    	// result(1,1) = cb;
    	// result(2,1) = cc;
      //
    	// float x1 = result(0,0);
    	// float y1 = result(1,0);
    	// float temp1, temp2, temp3;
      //
    	// temp1 = 1.0 + (x1*x1 / (y1*y1+EPS)) + (ca-cb*x1/(y1+EPS))*(ca-cb*x1/(y1+EPS))/(cc*cc+EPS);
    	// temp1 = sqrt(1.0 /(temp1+EPS));
    	// temp3 = -(ca - cb*x1/(y1+EPS)) / (cc *temp1+EPS);
    	// temp3 = temp3 >0 ? temp3: -1*temp3;
    	// result(2,2) = temp3;
    	// result(0,2) = -cc*temp3 /(ca-cb*x1/(y1+EPS)+EPS);
    	// result(1,2) = -x1/(y1 *result(0,2)+EPS);
      if (cc!=0){
      float temp = sqrt((cc*cc)/(ca*ca+cc*cc));
    	result(0,0) = temp >= 0 ? temp : -1*temp;
    	result(2,0) = - ca / cc *result(0,0);
    	result(1,0) = 0.0;
    	result(0,2) = ca;
    	result(1,2) = cb;
    	result(2,2) = cc;

    	float x1 = result(0,0);
    	float z1 = result(2,0);
    	float temp1, temp2, temp3;

      temp1 = 1.0 + (cb*cb)*(x1*x1+z1*z1)/((x1*cc-z1*ca)*(x1*cc-z1*ca));
      temp2 = sqrt(1.0/temp1);
      result(1,1) = temp2;
      result(0,1) = z1*cb/(x1*cc-z1*ca)*result(1,1);
      result(2,1) = (-x1*cb)/(x1*cc-z1*ca)*result(1,1);
      }
      //special case 1
      else if(cc==0 && cb !=0)
      {
        if(cb<0)
        {
          ca = -ca;
          cb = -cb;
          cc = -cc;
        }
        result(0,0) = 0.0;
        result(1,0) = 0.0;
        result(2,0) = 1.0;
        float temp = sqrt((cb*cb)/(ca*ca+cb*cb));
        result(0,1) = temp>=0? temp:-1*temp;
        result(1,1) = -ca / cb*result(0,1);
        result(2,1) = 0;
        result(0,2) = ca;
        result(1,2) = cb;
        result(2,2) = cc;
      }
      //special case 2
      else if(cc==0 && cb==0)
      {
        result(0,0) = 0.0;
        result(1,0) = 0.0;
        result(2,0) = 1.0;
        result(0,1) = 0.0;
        result(1,1) =-1.0;
        result(2,1) = 0.0;
        result(0,2) = 1.0;
        result(1,2) = 0.0;
        result(2,2) = 0.0;
      }



    	return result.inverse(); //注意需要转置

    }

    bool inPolygon(const Pointf_& target)
    {
      //TODO
      // vector<Pointf_> points_temp = m_points;
      // int size = m_points.size();
      //project 3D to 2D
      // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();  //TODO
      // clock_t start_temp, end_temp;
      // start_temp = clock();
      int size = m_points.size();
      float min_x, min_y, min_z, max_x, max_y, max_z;
      float temp1, temp2, temp3;
      float* x_temp = new float[size];
      float* y_temp = new float[size];
      float* z_temp = new float[size];

      for (size_t i=0; i<size; i++)
      {
        x_temp[i] = m_points[i](0);
        y_temp[i] = m_points[i](1);
        z_temp[i] = m_points[i](2);
      }
      //冒泡排序
      for(size_t i=1;i<size;i++)
      {
        for(size_t j=i;j>0;j--)
        {
            if(x_temp[j]<x_temp[j-1])
            {
              temp1 = x_temp[j-1];
              x_temp[j-1] = x_temp[j];
              x_temp[j] = temp1;
            }
            if(y_temp[j]<y_temp[j-1])
            {
              temp2 = y_temp[j-1];
              y_temp[j-1] = y_temp[j];
              y_temp[j] = temp2;
            }
            if(z_temp[j]<z_temp[j-1])
            {
              temp3 = z_temp[j-1];
              z_temp[j-1] = z_temp[j];
              z_temp[j] = temp3;
            }
        }
      }

      min_x = x_temp[0];
      max_x = x_temp[size-1];
      min_y = y_temp[0];
      max_y = y_temp[size-1];
      min_z = z_temp[0];
      max_z = z_temp[size-1];
      // simple test whether target is in Polygon or not.
      if(target(0)<(min_x-0.1) || target(0)>(max_x+0.1) || target(1)<(min_y-0.1)||
         target(1)>(max_y+0.1) || target(2)<(min_z-0.1) || target(2)>(max_z+0.1))
      {
        // cout<<"WARNNING3"<<endl;
        return false;
      }

      Eigen::Matrix4f transform;
      transform = calculate_trans(m_normal);

      // cout<<"<<"<<m_normal<<">>"<<endl;
      // cout<<transform<<endl;
      pcl::PointCloud<PointT_> cloudI;
      PointT_ pp;
      for(size_t i=0;i<m_points.size();i++)
      {
        pp.x = m_points[i](0);
        pp.y = m_points[i](1);
        pp.z = m_points[i](2);
        pp.intensity = i;
        cloudI.push_back(pp);
      }
      pp.x = target[0];
      pp.y = target[1];
      pp.z = target[2];
      pp.intensity = m_points.size();
      cloudI.push_back(pp);

      // 执行变换，并将结果保存在新创建的‎‎ trans_cloudI ‎‎中
      pcl::PointCloud<PointT_>::Ptr trans_cloudI (new pcl::PointCloud<PointT_>());
      pcl::transformPointCloud (cloudI, *trans_cloudI, transform);
      // end_temp = clock();
      // double period_temp = (double)(end_temp - start_temp) / CLOCK_PER_SEC;
      //cout<<"one transform take time of: "<<period_temp<<" s"<<endl;
      vector<Point2f_> points_temp;
      for (size_t i=0;i<trans_cloudI->points.size();i++)
      {
        // cout<<"["<<trans_cloudI->points[i].x<<","
        //     <<trans_cloudI->points[i].y<<","
        //     <<trans_cloudI->points[i].z<<"]"<<endl;
        Point2f_ pt(trans_cloudI->points[i].x, trans_cloudI->points[i].y); //TODO
        points_temp.push_back(pt);
      }
      Point2f_ targ_pt = points_temp.back();  //TODO
      // cout<<"target"<<targ_pt<<endl;
      points_temp.pop_back();

      // int size = points_temp.size();
      float* vertx = new float[size];
      float* verty = new float[size];
      // float* vertz = new float[size];
      for (size_t i=0; i<size; i++)
      {
        // x_temp[i] = points_temp[i](0);
        // y_temp[i] = points_temp[i](1);
        vertx[i] = points_temp[i](0);
        verty[i] = points_temp[i](1);
        // vertz[i] = points_temp[i](2);
      }

      //major test
      int in_out;
      // cout<<"size"<<size<<"vertx"<<vertx[0]<<"verty"<<verty[0]<<endl;
      in_out = InOrOutPolygon_2d(size,vertx,verty, targ_pt(0), targ_pt(1));

      return (in_out == 0 ? false: true);
    }

  /************************************************************
  ** 函数名称:  InOrOutPolygon_2d
  ** 功能描述:  判断点在多边形内外（不包含特殊情况如点在线上，射线过定点等）
  ** 输入参数:  nvert 顶点个数 vertx 多边形顶点x坐标数组 verty 多边形顶点y坐标数组
                testx 被判断点位置x坐标 testy 被判断点位置y坐标
  ** 输出参数:  NULL
  ** 返 回 值:  0:外 1:内
  **************************************************************/
    int InOrOutPolygon_2d(int nvert, float *vertx, float *verty, float testx, float testy)
    {
      int i, j, c = 0;
      for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
        (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
         c = !c;
       }

       // cout<<"crossing"<<c<<endl;
      return c;
    }

    Color getcolor() const
    {
      return m_color;
    }

    Eigen::Vector3f getnormal() const
    {
      return m_normal;
    }

    vector<Line> getlines() const  //external visit
    {
      return m_lines;
    }

    Line getline(int num) const  //external visit
    {
      return m_lines[num];
    }

    int getfacelabel() const
    {
      return m_label;
    }

    int getbuildlabel() const
    {
      return m_build_label;
    }

    void setbuildlabel(int label)
    {
      m_build_label = label;  //build label for face
      for (size_t i=0;i<m_lines.size();i++)
      {
        m_lines[i].setbuildlabel(label); //build label for line
      }
    }

    void setVisibility()
    {
      m_faceVisibility = true;
    }

    void setLineVisibility(int num, int visibility)
    {
      m_lines[num].m_visibility = visibility;
    }

    bool m_faceVisibility; //-1：不可见（default）　１：可见

  private:
    vector<Pointf_> m_points;
    vector<Line> m_lines;  //local container
    Color m_color;
    Eigen::Vector3f m_normal;
    int m_label;
    int m_build_label;
  };

  class Building : public Shape
  {
  public:
    Building(){}
    Building(const vector<Point2f_>& points_raw, float up, float down)
       : m_points(points_raw), m_up(up), m_down(down), m_label(-1)
    {
      vector<Point2f_> points = points_raw;
      points.push_back(points_raw[0]);  //build a cycle
      for (size_t i=0;i<points.size()-1;i++)
      {
        vector<Pointf_> facePointList;
        Pointf_ p1(points[i](0),points[i](1),up);
        Pointf_ p2(points[i+1](0),points[i+1](1),up);
        Pointf_ p3(points[i+1](0),points[i+1](1),down);
        Pointf_ p4(points[i](0),points[i](1),down);
        facePointList.push_back(p1);
        facePointList.push_back(p2);
        facePointList.push_back(p3);
        facePointList.push_back(p4);
        Eigen::Vector3f temp1, temp2;
        Eigen::Vector3f normal;
        temp1 = p1 - p2;
        temp2 = p3 - p2;

        normal = temp1.cross(temp2);
        normal.normalize();

        Color color;
        color.rand_normalcolor();
        Polygon wall(facePointList, normal, color, m_faces.size());

        m_faces.push_back(wall);
      }

      vector<Pointf_> ceilPointList, floorPointList;
      for (size_t i=0;i<points.size()-1;i++)
      {
        Pointf_ p_up(points[i](0),points[i](1),up);
        Pointf_ p_down(points[i](0),points[i](1),down);

        ceilPointList.push_back(p_up);
        floorPointList.push_back(p_down);
        // cout<<floorPointList[i]<<endl;
      }


      Eigen::Vector3f global_normal(0.0,0.0,1.0);
      Color color_;
      color_.rand_normalcolor();
      Polygon ceiling(ceilPointList,global_normal,color_,m_faces.size());
      m_faces.push_back(ceiling);
      color_.rand_normalcolor();
      Polygon floor(floorPointList,global_normal,color_, m_faces.size());
      m_faces.push_back(floor);

      color_.rand_normalcolor();
      m_color = color_;

      //计算boundingbox
      int size = m_points.size();
      float min_x, min_y, max_x, max_y;
      float temp1, temp2;
      float* x_temp = new float[size];
      float* y_temp = new float[size];

      for (size_t i=0; i<size; i++)
      {
        x_temp[i] = m_points[i](0);
        y_temp[i] = m_points[i](1);
      }
      //冒泡排序
      for(size_t i=1;i<size;i++)
      {
        for(size_t j=i;j>0;j--)
        {
            if(x_temp[j]<x_temp[j-1])
            {
              temp1 = x_temp[j-1];
              x_temp[j-1] = x_temp[j];
              x_temp[j] = temp1;
            }
            if(y_temp[j]<y_temp[j-1])
            {
              temp2 = y_temp[j-1];
              y_temp[j-1] = y_temp[j];
              y_temp[j] = temp2;
            }
        }
      }
      min_x = x_temp[0];
      max_x = x_temp[size-1];
      min_y = y_temp[0];
      max_y = y_temp[size-1];
      m_boundingbox.push_back(Point2f_(min_x,min_y));
      m_boundingbox.push_back(Point2f_(max_x,max_y));
      m_boxcenter = Point2f_((min_x+max_x)/2.0,(min_y+max_y)/2.0);
      addShapes();
    }

    ~Building(){}

    void addShapes()
    {
      for (size_t i=0;i<m_faces.size();i++)
      {
        Shape* pShape = &(m_faces[i]);
        m_shapes.push_back(pShape);
      }
    }

    virtual bool intersect(Intersection& intersection)
    {
      bool intersectedAny = false;
      for(list<Shape*>::iterator iter=m_shapes.begin();iter!=m_shapes.end();++iter)
      {
        Shape* pShape = *iter;
        bool intersected = pShape ->intersect(intersection);
        if(intersected)
        {
          intersectedAny = true;
        }
      }
      // int face_id = intersection.m_intersectedFace_label[1];
      // m_faces[face_id].setVisibility();
      // visibleFaces.push_back(m_faces[face_id]);
      return intersectedAny;
    }

    vector<Polygon> getfaces() const
    {
      return m_faces;
    }

    Color getcolor() const
    {
      return m_color;
    }

    virtual int getlabel()
    {
      return m_label;
    }

    virtual void setlabel(int label)
    {
      m_label = label;
      for (size_t i=0;i<m_faces.size();i++)
      {
        m_faces[i].setbuildlabel(label);
      }
    }

    virtual int* calculate_bounding_range(const Point2f_& origin)
    {
      int* temp =new int[4];
      int* result =new int[2];
      vector<Point2f_> corners;
      corners = m_boundingbox;
      corners.push_back(Point2f_(m_boundingbox[0](0),m_boundingbox[1](1)));
      corners.push_back(Point2f_(m_boundingbox[1](0),m_boundingbox[0](1)));
      assert(corners.size()==4);
      for(size_t i=0;i<4;i++)
      {
        float x,y;
        x = corners[i](0);
        y = corners[i](1);
        temp[i] = int((atan2(y-origin(1),x-origin(0)) + M_PI + 0.01) * 180.0 / M_PI);
      }
      int low_temp = min(temp[0],min(temp[1],min(temp[2],temp[3])));
      int high_temp = max(temp[0],max(temp[1],max(temp[2],temp[3])));

      if(high_temp -low_temp < 180)
      {
        result[0] = low_temp>=2? low_temp-2:low_temp;
        result[1] = high_temp+2;
      }
      else
      {
        int high = 360;
        int low = 0;
        for (size_t i=0;i>4;i++)
        {
          if(temp[i]>=180){
            if(high>temp[i]) high = temp[i];
          }
          else{
            if(low<temp[i]) low = temp[i];
          }
        }
        result[0] = low;
        result[1] = high;
      }

      return result;
    }

    virtual vector<Point2f_> getboundingbox()
    {
      return m_boundingbox;
    }

    virtual Point2f_ getboxcenter()
    {
      return m_boxcenter;
    }

    vector<Polygon> getPolygons() const
    {
      return m_faces;
    }

    Polygon getPolygon(int num) const //external visit
    {
      return m_faces[num];
    }

    vector<Point2f_> getPoints() const
    {
      return m_points;
    }

    float getUp() const
    {
      return m_up;
    }

    float getDown() const
    {
      return m_down;
    }

    void setPolygon(int num)
    {
      m_faces[num].setVisibility();
    }

    Building& operator =(const Building& build)
    {
      vector<Polygon> temp = build.getPolygons();
      m_faces.assign(temp.begin(),temp.end());
      m_points.assign(build.m_points.begin(),build.m_points.end());
      m_shapes = build.m_shapes;
      m_up = build.m_up;
      m_down = build.m_down;
      m_label = build.m_label;
      m_boundingbox.assign(build.m_boundingbox.begin(),build.m_boundingbox.end());
      m_boxcenter = build.m_boxcenter;
      m_color = build.m_color;
      return *this;
    }
  private:
    vector<Point2f_> m_points;
    vector<Polygon> m_faces;  //local container
    list<Shape*> m_shapes;
    float m_up;
    float m_down;
    int m_label;
    vector<Point2f_> m_boundingbox;
    Point2f_ m_boxcenter;
    Color m_color;
  };

  //define derived class
  class BuildingSet : public Shape
  {
  public:
    virtual ~BuildingSet(){}

    virtual bool intersect(Intersection& intersection)
    {
      bool intersectedAny = false;
      for(vector<Shape*>::iterator iter=m_buildings_pick.begin(); iter!=m_buildings_pick.end();++iter)
      {
        Shape* pBuild = *iter;
        bool intersected  = pBuild->intersect(intersection);
        if(intersected)
        {
          intersectedAny = true;
        }
      }
      int build_id = intersection.m_intersectedFace_label[0];
      int face_id = intersection.m_intersectedFace_label[1];
      if (build_id>=0 && face_id>=0)
      {
        m_buildings_local[build_id].setPolygon(face_id);
        // m_buildings[build_id]->   //TODO
      }

      return intersectedAny;
    }

    void addBuilding(Building* pShape)
    {
      int label = m_buildings.size(); //building label
      pShape->setlabel(label);
      m_buildings.push_back(pShape);
      m_buildings_local.push_back(*pShape);
    }

    void buildTree(const Pointf_& origin)
    {
      // delete m_rbt;
      //TODO
      Point2f_ origin_2d(origin(0),origin(1));
      vector<typename rb_tree::rb_interval> inputs;
      // int n=0;
      // cout<<"start iteration"<<endl;
      for (vector<Shape*>::iterator iter=m_buildings.begin(); iter!=m_buildings.end();++iter)
      {
        // cout<<"looping at:"<<endl;
        Shape* pBuild = *iter;
        int* angle = pBuild->calculate_bounding_range(origin_2d);
        /*TODO
          计算一定范围(tree_range)内建筑bounding box边界到光心的角度range构建rbtree
        */
        Point2f_ buildcenter = pBuild->getboxcenter();
        double distance = sqrt((buildcenter(0)-origin_2d(0))*(buildcenter(0)-origin_2d(0))
                              +(buildcenter(1)-origin_2d(1))*(buildcenter(1)-origin_2d(1)));
        if(distance > tree_range) continue;  //超过tree_range

        if (angle[1] - angle[0] > 180)
        {
          int temp1 = angle[0];
          int temp2 = angle[1];
          angle[0] = temp2;
          angle[1] = 360;
          typename rb_tree::rb_interval tempp1(angle[0],angle[1],pBuild->getlabel(),false);
          inputs.push_back(tempp1);
          angle[0] = 0;
          angle[1] = temp1;
          typename rb_tree::rb_interval tempp2(angle[0],angle[1],pBuild->getlabel(),false);
          inputs.push_back(tempp2);
        }
        else
        {
          typename rb_tree::rb_interval temp(angle[0],angle[1],pBuild->getlabel(),false);
          inputs.push_back(temp);
        }
        // cout<<angle[0]<<"   "<<angle[1]<<"  "<<pBuild->getlabel()<<endl;

        // n++;
      }

      if (inputs.size()==0)
      {
        cout<<"WARNNING: Distance between buildings and camera is bigger than <tree_range: "
                <<tree_range<<" >"<<"shutting down now..." <<endl;
        exit(1);
      }

      if(print_mode){
        for(size_t i=0;i<inputs.size();i++)
        {
          if(i==0){
            cout<<"Buildings in the tree: "<<endl;
          }
          cout<<inputs[i].label<<" "<<inputs[i].low<<" "<<inputs[i].high<<endl;;
        }
      }

      m_rbt = new rb_tree(inputs,inputs.size());
      cout<<"rb tree constructed"<<endl;
     //  typename rb_tree::rb_interval target(10,350);
     //  while(m_rbt->rb_search(target)!=NULL)
     //  {
     //  typename rb_tree::prb_type result = m_rbt->rb_search(target);
     //  cout<<"left:" <<result->inte.low <<"right :"<<result->inte.high<<endl;
     //  m_rbt->rb_delete(result->inte);
     // }
      // result = rbt.rb_search(target);
      // cout<<"left:" <<result->inte.low <<"right :"<<result->inte.high<<endl;
    }
    void pickBuilding(Intersection& intersection)
    {
      //TODO
      clearPick();
      // rb_tree* rbt_temp = m_rbt;
      float ray_angle = intersection.m_ray.m_angle;
      typename rb_tree::rb_interval target((int)ray_angle,(int)ray_angle,-1, false);
      // cout<<"ray_angle"<<ray_angle<<endl;
      // while(m_rbt->rb_search_multi(target)!=m_rbt->rb_nil())
      //  {
      //    typename rb_tree::prb_type result = m_rbt->rb_search_multi(target);
      //    if (print_mode){
      //    cout<<"left:" <<result->inte.low <<"right :"<<result->inte.high<<
      //    "label:"<<result->inte.label<<endl;
      //    }
      //    result->inte.visited = true;
      //    // m_rbt->rb_delete(result->inte);
      //    m_buildings_pick.push_back(m_buildings[result->inte.label]);
      //  }
       vector<typename rb_tree::prb_type> result = m_rbt->rb_search_multi(target);
       for(size_t i=0;i<result.size();i++)
       {
         typename rb_tree::prb_type res = result[i];
         if(res!=m_rbt->rb_nil())
         {
           if (print_mode){
             cout<<"left:" <<res->inte.low <<"right :"<<res->inte.high<<
             "label:"<<res->inte.label<<endl;
             }
           m_buildings_pick.push_back(m_buildings[res->inte.label]);
         }
       }
       // cout<<"picked size: "<<m_buildings_pick.size()<<endl;
       // m_rbt->cleanvisited(m_rbt->rb_root());  //O(log(n))
    }

    Building getBuilding(int num) const  //external visit
    {
      return m_buildings_local[num];
    }

    vector<int> getPickedBuildingLabel()
    {
      vector<int> res;
      for(vector<Shape*>::iterator iter=m_buildings_pick.begin(); iter!=m_buildings_pick.end();++iter)
      {
        Shape* pBuild = *iter;
        res.push_back(pBuild->getlabel());
      }
      return res;
    }

    int size() const
    {
      return m_buildings_local.size();
    }

    void clearBuilding()
    {
      m_buildings.clear();
    }
    void clearPick()
    {
      m_buildings_pick.clear();
    }
    void clearTree()
    {
      m_rbt = NULL;
    }

  protected:
    vector<Shape*> m_buildings;
    vector<Shape*> m_buildings_pick;
    vector<Building> m_buildings_local;  //local container, only for external visit
    rb_tree* m_rbt;
  };


}//namespace DosuRay
#endif
