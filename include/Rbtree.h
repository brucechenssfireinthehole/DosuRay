/*
　　区间树 RB-tree
   date:2019-08-08
*/
#ifndef RBTREE_H
#define RBTREE_H
//sys
#include <iostream>
#include <sstream>
#include <deque>
#include <iomanip>
#include <vector>
#include <stack>

#define SENTINEL -100000

using namespace std;

class rb_tree {//区间树
public:
      typedef struct _rb_interval {
          int low;
          int high;
          int label;
          bool visited;
          _rb_interval(){}
          _rb_interval(int _low, int _high,int _label, bool _visited=false):low(_low),
                        high(_high), label(_label), visited(_visited){}
          _rb_interval& operator =(const _rb_interval& c)
          {
            low = c.low;
            high = c.high;
            label = c.label;
            visited = c.visited;
          }
      }rb_interval, *prb_interval;

      typedef struct _rb_type {
          _rb_type(){}
          _rb_type(_rb_type *_left, _rb_type *_right, _rb_type *_p, bool cl, _rb_interval _inte) :
              left(_left), right(_right), p(_p), color(cl), inte(_inte), max(_inte.high) {}
          bool color;//true for red, false for black
          int max;//区间上限
          rb_interval inte;//区间范围
          _rb_type *left, *right, *p;
      }rb_type, *prb_type;

      rb_tree() : root(NULL){}
      rb_tree(vector<rb_interval> A, int n) :root(NULL) {
          for (int i = 0; i < n; i++)
              this->rb_insert(A[i]);
      }
      rb_tree& operator =(rb_tree& rb)
      {
        root = rb.root;
      }
      ~rb_tree() {
          rb_empty(root);
      }
      void left_rotate(prb_type x);
      void right_rotate(prb_type x);
      void rb_insert(rb_interval _inte);
      prb_type rb_max(prb_type x);
      prb_type rb_min(prb_type x);
      prb_type rb_search(rb_interval _inte);//《算法导论》给出的重叠查找
      prb_type rb_search_exact(rb_interval _inte);//精确查找，删除节点需要
      vector<prb_type> rb_search_multi(rb_interval _inte);//实际使用
      prb_type rb_next(rb_interval _inte);
      prb_type rb_prev(rb_interval _inte);
      void rb_delete(rb_interval _inte);
      void rb_empty(prb_type x);//后续全部删除
      void cleanvisited(prb_type x);
      prb_type rb_root();
      prb_type rb_nil();
      void rb_show(prb_type x);
private:
      bool overlap(rb_interval _x, rb_interval _y);
      int max(int a, int b, int c);
      int max(int a, int b);
      void rb_insert_fixup(prb_type x);
      void rb_delete_fixup(prb_type x);
      //测试使用
      int rb_max_depth(prb_type x);
      int rb_min_depth(prb_type x);
      prb_type root;
      prb_type nil;
};

#endif //RBTREE_H
