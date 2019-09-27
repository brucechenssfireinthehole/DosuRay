/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#ifndef CIRCLELINKEDLIST_H
#define CIRCLELINKEDLIST_H

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

struct PolyCorner
{
  float x;
  float y;
  PolyCorner():x(0),y(0){}
  PolyCorner(float a, float b):x(a),y(b){}
  PolyCorner& operator =(const PolyCorner& c)
  {
    x = c.x;
    y = c.y;
    return *this;
  }
  PolyCorner& operator -=(const PolyCorner& c)
  {
    x-=c.x;
    y-=c.y;
    return *this;
  }
};

inline PolyCorner operator -(const PolyCorner& c1, const PolyCorner& c2)
{
  return PolyCorner(c1.x-c2.x, c1.y-c2.y);
}

struct node{
	PolyCorner corner;
  node* prev;
	node* next;
};

class CircleLinkedList
{
public:
  CircleLinkedList():root(NULL),length(0){}
  ~CircleLinkedList(){}

  bool isEmpty() const
  {
    return(length==0? true:false);
  }

  int returnLength() const
  {
    return length;
  }

  node* returnRoot() const
  {
    return root;
  }
  // bool insert(int position, PolyCorner corner);
  void headInsert(PolyCorner cor);
  void rearInsert(PolyCorner cor);
  PolyCorner returnCorner(int position);
  // void returnAllCorner(int position);
  void deleteNode(node* n);
  void unique();
  // bool deleteAll();

private:
  node* root;
  int length;
};

#endif //CircleLinkedList.h
