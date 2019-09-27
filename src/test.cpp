#include "Common.h"
//support Rbtree
#include "Rbtree.h"

int data[40][3]={{11,301,311},{22,306,322},{41,292,328},{54,224,244},
               {58,245,260},{67,301,314},{73,327,340},{77,323,335},
               {78,294,304},{82,296,333},{85,285,320},{88,255,275},
               {92,227,312},{102,313,340},{104,222,243},{137,289,305},
               {139,283,301},{143,280,297},{147,285,299},{151,281,296},
               {152,259,274},{153,226,246},{154,299,312},{155,301,315},
               {164,312,336},{171,278,296},{181,260,276},{183,327,354},
               {184,263,278},{194,295,310},{196,291,309},{204,267,278},
               {219,310,331},{222,274,282},{226,302,317},{227,287,302},
               {230,299,319},{231,304,313},{232,308,321},{234,307,320}
             };

int main()
{
  vector<typename rb_tree::rb_interval> inputs;

  for(int i=0;i<40;i++)
  {
    typename rb_tree::rb_interval temp(data[i][1],data[i][2],data[i][0],false);
    inputs.push_back(temp);
  }
  // typename rb_tree::rb_interval temp0(1,362,0,false);
  // inputs.push_back(temp0);
  // typename rb_tree::rb_interval temp1(10,25,1,false);
  // inputs.push_back(temp1);
  // typename rb_tree::rb_interval temp2(139,199,2,false);
  // inputs.push_back(temp2);
  // typename rb_tree::rb_interval temp3(120,223,3,false);
  // inputs.push_back(temp3);
  // typename rb_tree::rb_interval temp4(166,251,4,false);
  // inputs.push_back(temp4);
  // typename rb_tree::rb_interval temp5(166,179,5,false);
  // inputs.push_back(temp5);

  rb_tree* m_rbt = new rb_tree(inputs,inputs.size());

  m_rbt->rb_show( m_rbt->rb_root());
  typename rb_tree::prb_type x = m_rbt->rb_root();

  typename rb_tree::rb_interval target(260,260,-1, false);
  vector<typename rb_tree::prb_type> result = m_rbt->rb_search_multi(target);
  for(size_t i=0;i<result.size();i++)
   {
     typename rb_tree::prb_type res = result[i];
     cout<<"label: "<<res->inte.label<<"left: " <<res->inte.low <<"right: "<<res->inte.high<<endl;
     // result->inte.visited = true;
     // m_rbt->rb_delete(result->inte);
     // m_buildings_pick.push_back(m_buildings[result->inte.label]);
   }
   return 1;
}
