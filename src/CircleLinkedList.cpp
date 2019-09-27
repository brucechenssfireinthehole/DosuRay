/*
This is C++ version of DosuRay package, which is used for Ray tracing in 3d-virtual
environment to obtain the hit plane and visable lines of 3d-object.
notice: any question please contact DOSU ${cbbsjtu@126.com}
*/

#include "CircleLinkedList.h"

// bool CircleLinkedList::insert(int position, PolyCorner corner){
//   if(position>length+1)
//     return false;
//   else{
//     node* probe=&data;
//     for(int i=1;i<position;i++){
//       probe=probe->next;
//     }
//     node* newnode=new node;
//     newnode->data1=num;
//     newnode->next=probe->next;
//     probe->next=newnode;
//     length++;
//   }
//   return true;
// }

void CircleLinkedList::headInsert(PolyCorner cor){
  node* start = root;
  node* end = root->prev;
  node* newnode=new node;
  newnode->corner = cor;

  newnode->next = start;
  newnode->prev =end;
  start->prev = newnode;
  end->next =newnode;

  root = newnode;
  length++;
}

void CircleLinkedList::rearInsert(PolyCorner cor){
  if(root==NULL)
  {
    root =new node;
    root->corner =cor;
    root->prev =root;
    root->next =root;
  }
  else
  {
    node* start = root;
    node* end = root->prev;
    node* newnode =new node;
    newnode->corner = cor;
    end->next =newnode;
    newnode->prev =end;
    newnode->next =start;
    start->prev =newnode;
  }
  length++;
}

PolyCorner CircleLinkedList::returnCorner(int position){
  node* start =root;
  for(int i=0;i<position;i++){
    start=start->next;
  }
  return start->corner;
}

// void CircleLinkedList::returnallnum(int position){
//   int times=0;
//   node* probe=&data;
//   for(int i=1;i<=position;i++){
//     probe=probe->next;
//   }
//   while(times<length){
//     if(probe!=(&data)){
//
//     probe=probe->next;
//   }
//     else
//     probe=probe->next;
//   }
// }

void CircleLinkedList::deleteNode(node* x){
  if(x!=root)
  {
    x->prev->next = x->next;
    x->next->prev = x->prev;
    x->next = NULL;
    x->prev = NULL;
  }
  else{
    root = root->next;
    root->prev->prev->next = root;
    root->prev = root->prev->prev;
    x->next = NULL;
    x->prev = NULL;
  }
  length--;
  // return true;
}

// bool CircleLinkedList::deleteall(){
//   node* probehead=&data;
//   node* proberear=NULL;
//   while(probehead->next!=&data){
//     proberear=probehead->next;
//     free(probehead);
//     probehead=proberear;
//     length--;
//   }
//   free(probehead);
//   if(length==0)
//     return true;
//   else
//     return false;
// }
void CircleLinkedList::unique()
{
  if(this->returnLength()<3)
  {
    cout<<"Polygon point number less than 3, shut down now..."<<endl;
    return;
  }
  //move root to an end point;
  double root_theta;
  do{
    node* root_n =root->next;
    node* root_p =root->prev;
    PolyCorner rn,rp;
    rn = root_n->corner - root->corner;
    rp = root->corner - root_p->corner;
    root_theta = acos((rn.x*rp.x+rn.y*rp.y)/(sqrt(rn.x*rn.x+rn.y*rn.y)*sqrt(rp.x*rp.x+rp.y*rp.y)));
    if(root_theta<(20.0*3.141593/180.0)){
      cout<<"move root to corner with root_theta-> "<<root_theta<<endl;
      root =root->prev;
    }
  }
  while(root_theta<(20.0*3.141593/180.0));

  node* x =root;
  while(x->next!=root)
  {
    double theta;
    do
    {
      node* x1 =x->next;
      node* x2 =x->next->next;
      PolyCorner c1,c2;
      c1 = x1->corner - x->corner;
      c2 = x2->corner - x1->corner;
      theta = acos((c1.x*c2.x+c1.y*c2.y)/(sqrt(c1.x*c1.x+c1.y*c1.y)*sqrt(c2.x*c2.x+c2.y*c2.y)));
      if(theta<(20.0*3.141593/180.0)){
        cout<<"delete a point which is not corner with theta->"<<theta<<endl;
        deleteNode(x1);
      }
    }
    while(theta<(20.0*3.141593/180.0));

    x = x->next;
  }

}
