/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef DATAREADER_H
#define DATAREADER_H

// #include <io.h>
#include "DosuRay.h"
#include "json/json.h"
#include "CircleLinkedList.h"

using namespace std;

namespace DosuRay
{


  class BuildingReader
  {
  public:
    BuildingReader(){}
    BuildingReader(const string& inputdir, double* origin_lon_lat, const Pointf_& camera_origin):
        m_camera_origin(camera_origin),
        m_inputdir(inputdir)
    {
      string inputfile = m_inputdir +"rows.json";
      m_inputfile = inputfile;
      string pickupfile = m_inputdir +"pickup.txt";
      m_pickupfile = pickupfile;
      // origin = to_string(m_camera_origin(0)) + "_" +
      //          to_string(m_camera_origin(1)) + "_" +
      //          to_string(m_camera_origin(2)) + ".txt";
      ifstream is(inputfile);
      if(!is.is_open()){
        cout<<"ERROR: open json file failed. Cannot without .json file, shut down now."<<endl;
        exit(1);
      }

      ifstream pickup(pickupfile);
      if(!pickup.good())
      {
        printf(" pickupfile not found. Creating pickupfile.txt now......\n");
        ofstream outFile(pickupfile,ios_base::ate|ios_base::out);
        outFile.setf(ios_base::fixed);
        outFile.precision(6);
        // Json::Reader reader;
        Json::Value root;
        is >> root;
        cout<<"Data parse finished."<<endl;
        // assert(reader.parse(is,root));
        Json::Value data;
        data = root["data"];
        for (int index=0;index<data.size();index++)
        {
          int bin = std::stoi(data[index][11].asString());
          string footprints = data[index][8].asString();
          vector<Point2d_> ss = strToPoint2d(footprints); //long-lat need convert to float
          for (size_t i=0;i<ss.size();i++)
          {
            lonlatToXY(ss[i],origin_lon_lat);
          }
          double groundelev = std::stod(data[index][17].asString());
          double heightroof = std::stod(data[index][15].asString());

          double error;
          Point2d_ p0 = ss[0];
          // lonlatToXY(p0,origin_lon_lat);
          error = sqrt((p0(0)-m_camera_origin(0)) *(p0(0)-m_camera_origin(0))
                      + (p0(1)-m_camera_origin(1)) * (p0(1)-m_camera_origin(1)));
          if(error<pickup_range)
          {
            outFile<<"build:"<<bin<<" X:";
            for (size_t i=0;i<ss.size();i++)
            {
              outFile<<ss[i](0)<<" ";
            }
            outFile<<"Y:";
            for (size_t i=0;i<ss.size();i++)
            {
              outFile<<ss[i](1)<<" ";
            }
            outFile<<"up:"<<heightroof+groundelev<<" down:"<<groundelev<<endl;
          }
          else{
            continue;
          }
        }
        outFile.close();
        cout<<"pickupfile created"<<endl;
      }

      // ifstream pickup_real(pickupfile);


      addBuildings();
    }

    ~BuildingReader(){}

    void addBuildings()
    {
      /*
        read from m_inputfile
      */
      cout<<"start loading data from -> "<<m_pickupfile<<endl;
      ifstream in(m_pickupfile);
      if (!in.good())
      {
        printf(" ... not found. Cannot operate without inputfile, shutting down.\n");
        return;
      }
      string line;
      while(getline(in,line))
      {
        string label, up, down;
        float label_f, up_f, down_f;
        vector<Point2f_> points_raw;
        vector<Point2f_> points_raw_real;
        vector<float> px_f, py_f;

        stringstream ss(line);
        string tmp;
        int i = 0;
        while(getline(ss,tmp,':'))
        {
          stringstream sss(tmp);
          if(i==0) {
            // cout<<tmp<<endl;
            i++;
            continue;
          }
          else if(i==1){
            // stringstream sss(tmp);
            // string label;
            getline(sss,label,' ');  //build label
            // cout<<label<<endl;
            label_f = stof(label);
            cout<<"label:"<<label<<" ";
          }
          else if(i==2){
            string px;
            // vector<float> px_f;
            while(sss>>px)
            {
              if(px!="Y")
              {
                px_f.push_back(stof(px));
              }
            }
            cout<<"X.size: "<<px_f.size()<<" ";
          }
          else if(i==3){
            string py;
            while(sss>>py)
            {
              if(py!="up")
              {
                py_f.push_back(stof(py));
              }

            }
            cout<<"Y.size: "<<py_f.size()<<" ";
          }
          else if(i==4){
            getline(sss,up,' ');
            up_f = stof(up);
            cout<<"up: "<<up<<" ";
          }
          else {
            if(i!=5)
            {
              cout<<"ERROR: some thing wrong with input.txt, most likely due to the input struct error."<<endl;
              return;
            }
            getline(sss,down);
            down_f = stof(down);
            cout<<"down: "<<down<<endl;
          }
          i++;
        }

        if(px_f.size()!=py_f.size())
        {
          cout<<"ERROR: input x-array and y-array size don't match"<<endl;
          return;
        }
        /* 条件判断：是否在camera_origin附近范围内*/
        //TODO
        if(true)
        {
          int size = px_f.size();
          for(int k=0;k<size;k++)
          {
            Point2f_ pt(px_f[k],py_f[k]);
            points_raw.push_back(pt);
          }
          /*
            构建双向循环链表: 剔除polygon中平行的邻边.
          */
          CircleLinkedList pointList;
          for (size_t p=0;p<points_raw.size();p++)
          {
            PolyCorner temp(points_raw[p](0),points_raw[p](1));
            pointList.rearInsert(temp);
          }

          //due to the end data repeat with start in json file;
          //if not repeat, please commit the next sentence.
          pointList.deleteNode(pointList.returnRoot()->prev);

          pointList.unique();
          // node* rot =pointList.returnRoot();
          // pointList.deleteNode(rot->prev);
          for(size_t p=0;p<pointList.returnLength();p++)
          {
            if(1){
              cout<<"x->"<<pointList.returnCorner((int)p).x
                  <<"  y->"<<pointList.returnCorner((int)p).y<<endl;
            }
            points_raw_real.push_back(Point2f_(pointList.returnCorner((int)p).x,
                                              pointList.returnCorner((int)p).y));
          }

          Building build_temp(points_raw_real, up_f, down_f);
          m_buildings.push_back(build_temp);

        }
      }
      // for (size_t n=0;n<buildings.size();n++)
      // {
      //   set.addBuilding(&buildings[n]);
      // }
      // cout<<"set size:"<<set.size()<<endl;
    }

    vector<Building> m_buildings;
  private:
    string m_inputdir;
    string m_inputfile;
    string m_pickupfile;
    Pointf_ m_camera_origin;
    // vector<Building> buildings;
    // BuildingSet m_masterset;
  };
}
#endif //DataReader.h
