/*
　　区间树 RB-tree
   date:2019-08-08
*/
#include "Rbtree.h"

using namespace std;

void rb_tree::left_rotate(typename rb_tree::prb_type x) {
    // cout<<"    left_rotate:start"<<endl;
    if(x->right!=nil)
    {
      prb_type y = x->right;//y非空
      x->right = y->left;
      if (y->left!=nil) y->left->p = x;//交换子节点
      y->p = x->p;//更新父节点
      if (x->p == nil)//将y连接到x的父节点
          root = y;
      else {
          if (x == x->p->left)
              x->p->left = y;
          else
              x->p->right = y;
      }
      y->left = x;
      x->p = y;
      //阶段二更新max
      // cout<<"    update max"<<endl;
      x->max = this->max(x->inte.high, x->left->max, x->right->max);
      y->max = this->max(y->inte.high, y->left->max, y->right->max);
      // cout<<"    left_rotate:end"<<endl;
    }
    else{
      cout<<"ERROR: can't left rotate,because no right child"<<endl;
    }
}

void rb_tree::right_rotate(typename rb_tree::prb_type x) {
  if(x->left!=nil)
  {
    prb_type y = x->left;
    x->left = y->right;
    if (y->right!=nil) y->right->p = x;
    y->p = x->p;
    if (x->p == nil)
        root = y;
    else {
        if (x == x->p->left)
            x->p->left = y;
        else
            x->p->right = y;
    }
    y->right = x;
    x->p = y;
    //阶段二更新max
    x->max = this->max(x->inte.high, x->left->max, x->right->max);
    y->max = this->max(y->inte.high, y->left->max, y->right->max);

  }
  else{
    cout<<"ERROR: can't right rotate,because no left child"<<endl;
  }

}

typename rb_tree::prb_type rb_tree::rb_max(typename rb_tree::prb_type x) {
    if (x == NULL) return NULL;
    while (x->right) x = x->right;
    return x;
}

typename rb_tree::prb_type rb_tree::rb_min(typename rb_tree::prb_type x) {
    if (x == NULL) return NULL;
    while (x->left) x = x->left;
    return x;
}

typename rb_tree::prb_type rb_tree::rb_search(typename rb_tree::rb_interval _inte) {
    prb_type x = root;
    while (x!=nil && !overlap(_inte, x->inte)) {
        if (x->left!=nil && x->left->max >= _inte.low)
            x = x->left;
        else
            x = x->right;
    }
    return x;
}

typename rb_tree::prb_type rb_tree::rb_search_exact(typename rb_tree::rb_interval _inte) {
    prb_type x = root;
    while (x!=nil && !(x->inte.low == _inte.low && x->inte.high == _inte.high)) {
        if (_inte.low < x->inte.low)
            x = x->left;
        else
            x = x->right;
    }
    return x;
}

vector <typename rb_tree::prb_type> rb_tree::rb_search_multi(typename rb_tree::rb_interval _inte) {
    vector <typename rb_tree::prb_type> result;
    prb_type x = root;
    stack<prb_type> rbtStack;
    rbtStack.push(x);
    prb_type node;
    while(!rbtStack.empty())
    {
      node =rbtStack.top();
      if(node!=nil && overlap(_inte,node->inte) && node->inte.visited==false){
        node->inte.visited = true;
        result.push_back(node);
      }
      else
      {
        rbtStack.pop();
        if(node->right!=nil&&node->right->max >=_inte.low){
          rbtStack.push(node->right);
        }
        if(node->left!=nil&&node->left->max >=_inte.low){
          rbtStack.push(node->left);
        }
      }
    }
    cleanvisited(root);

    return result;
    // while ((x!=nil && !overlap(_inte, x->inte)) || (x!=nil && x->inte.visited==true)) {
    //     if (x->left!=nil && x->left->max >= _inte.low)
    //     {
    //       x = x->left;
    //     }
    //     else
    //     {
    //       x = x->right;
    //     }
    //
    // }
    // cout<<"search once"<<endl;
    // return x;
}

typename rb_tree::prb_type rb_tree::rb_next(typename rb_tree::rb_interval _inte) {
    prb_type x = rb_search_exact(_inte), y;
    if (x == NULL) return NULL;
    if (x->right)
        return rb_min(x->right);
    y = x->p;
    while (y != NULL && y->right == x) {//没有则返回NULL
        x = y;
        y = y->p;
    }
    return y;
}

typename rb_tree::prb_type rb_tree::rb_prev(typename rb_tree::rb_interval _inte) {
    prb_type x = rb_search_exact(_inte), y;
    if (x == NULL) return NULL;
    if (x->left)
        return rb_max(x->left);
    y = x->p;
    while (y != NULL && y->left == x) {
        x = y;
        y = y->p;
    }
    return y;
}

void rb_tree::rb_insert(typename rb_tree::rb_interval _inte) {
    if(root ==NULL)
    {
      nil = new rb_type;
      root = new rb_type(nil,nil,nil,false,_inte);

      // nil->p = root;
      nil->left = root;
      nil->right = root;
      nil->inte.low = nil->inte.high = SENTINEL;
      nil->color = false;
      nil->max = 0;
    }
    else{
      prb_type y = nil, x = root, z = new rb_type(nil, nil, nil, true,_inte);
      while (x != nil) {
          // cout<< "insert->1"<<endl;
          y = x;
          // x->max = this->max(x->max, z->max);//阶段一更新max
          if (_inte.low < x->inte.low)
              x = x->left;
          else
              x = x->right;
      }
      //更新max
      z->max = this->max(z->max,z->left->max,z->right->max);
      z->p = y;

      if (_inte.low < y->inte.low)
        y->left = z;
      else
        y->right = z;
      //再次更新max
      y->max = this->max(y->max,y->left->max,y->right->max);
      rb_insert_fixup(z);
    }
}

void rb_tree::rb_insert_fixup(typename rb_tree::prb_type x) {
    prb_type y;
    while (x->p->color) {//红色
        if (x->p == x->p->p->left) {//父节点存在，一定存在祖父节点
            y = x->p->p->right;
            //无法满足性质4
            if (y->color) {//若y为NULL，默认不存在的节点是黑色
                x->p->color = false;
                y->color = false;
                x->p->p->color = true;
                x = x->p->p;//重新设置z节点
            }
            else
            {
              if (x == x->p->right) { //无法满足性质5
                  x = x->p;
                  left_rotate(x);
              }

              x->p->color = false;
              x->p->p->color = true;
              right_rotate(x->p->p);
            }
        }
        else {//和上面左节点相反
            y = x->p->p->left;
            if (y->color) {
                x->p->color = false;
                y->color = false;
                x->p->p->color = true;
                x = x->p->p;//重新设置z节点
            }
            else
            {
                if (x == x->p->left)
                {
                  // cout<<"    2-2:start"<<endl;
                  x = x->p;
                  right_rotate(x);
                  // cout<<"    2-2:end"<<endl;
                }
                // cout<<"    2-3:start"<<endl;
                x->p->color = false;
                x->p->p->color = true;
                left_rotate(x->p->p);
                // cout<<"    2-3:end"<<endl;
            }
        }
    }
    // cout<<"      ->rb_insert_fixup_once"<<endl;
    root->color = false;
    root->p = nil;

    nil->left = root;
    nil->right = root;
}

void rb_tree::rb_delete(typename rb_tree::rb_interval _inte) {
    prb_type z = rb_search_exact(_inte), y, x;
    if (z == NULL) return;
    if (z->left == NULL || z->right == NULL)//y是待删除的节点
        y = z;//z有一个子节点
    else
        y = rb_next(_inte);//z有两个子节点，后继和前趋保证了y有一个或没有子节点
    if (y->left != NULL)
        x = y->left;
    else
        x = y->right;
    if (x != NULL) //存在一个子节点，先更正父子关系
        x->p = y->p;
    if (y->p == NULL)//再决定是在左或者右节点
        root = x;
    else {
        if (y->p->left == y)
            y->p->left = x;
        else
            y->p->right = x;
    }
    if (y != z)//处理两个子节点的交换
        z->inte = y->inte;
    //更新max
    z = y->p;
    while (z) {
        z->max = this->max(z->max, z->left ? z->left->max : 0, z->right ? z->right->max : 0);
        z = z->p;
    }
    if (!y->color)//黑色
        rb_delete_fixup(x);
    delete y;
}

void rb_tree::rb_delete_fixup(typename rb_tree::prb_type x) {
    prb_type w;
    while (x && x != root && !x->color) {//黑色
        if (x == x->p->left) {
            w = x->p->right;
            if (w->color) {//红色
                w->color = false;
                x->p->color = true;
                left_rotate(x->p);
                w = x->p->right;
            }
            if ((!w->left && !w->right) || (!w->left->color && !w->right->color)) {//双黑
                w->color = true;
                x = x->p;
            }
            else {
                if (!w->right->color) {//单黑
                    w->left->color = false;
                    w->color = true;
                    right_rotate(w);
                    w = x->p->right;
                }
                w->color = x->p->color;
                x->p->color = false;
                w->right->color = false;
                left_rotate(x->p);
                x = root;
            }
        }
        else {//相反的情况
            w = x->p->left;
            if (w->color) {//红色
                w->color = false;
                x->p->color = true;
                right_rotate(x->p);
                w = x->p->left;
            }
            if ((!w->left && !w->right) || (!w->left->color && !w->right->color)) {//双黑
                w->color = true;
                x = x->p;
            }
            else {
                if (!w->left->color) {//单黑
                    w->right->color = false;
                    w->color = true;
                    left_rotate(w);
                    w = x->p->left;
                }
                w->color = x->p->color;
                x->p->color = false;
                w->left->color = false;
                right_rotate(x->p);
                x = root;
            }
        }
    }
    if (x) x->color = false;//巧妙处理，默认黑
}

void rb_tree::rb_empty(typename rb_tree::prb_type x) {
    if (x != NULL) {
        rb_empty(x->left);
        rb_empty(x->right);
        printf("\n--------------[%d,%d]---------\n",x->inte.low,x->inte.high);
        rb_delete(x->inte);//后续保证子叶为空
        rb_show(root);
    }
}

void rb_tree::cleanvisited(typename rb_tree::prb_type x){
  prb_type x_left, x_right;
  if(x!=nil)
  {
    if(x->inte.visited)
      x->inte.visited = false;
    x_left = x->left;
    x_right = x->right;
    cleanvisited(x_left);
    cleanvisited(x_right);
  }
  else
    return;
}

typename rb_tree::prb_type rb_tree::rb_root() {
    return root;
}

typename rb_tree::prb_type rb_tree::rb_nil() {
    return nil;
}


bool rb_tree::overlap(typename rb_tree::rb_interval _x, typename rb_tree::rb_interval _y) {//闭区间
    if (_x.high < _y.low || _x.low > _y.high)     // _x 和 _y 没有重叠
        return false;
    return true;
}

int rb_tree::max(int a, int b, int c) {
    if (a>b)
        return a>c ? a : c;
    else
        return b>c ? b : c;
}

int rb_tree::max(int a, int b) {
    return a > b ? a : b;
}

void rb_tree::rb_show(typename rb_tree::prb_type x) {
    if (x != nil) {
        rb_show(x->left);
        if (x == root)
            printf("root: (%s)[%d,%d], max=%d, (%d,%d)\n", root->color ? "red" : "black", x->inte.low, x->inte.high, x->max,
                rb_max_depth(x), rb_min_depth(x));
        else
            printf("(%s)[%d,%d], max=%d, (%d,%d)\n", x->color ? "red" : "black", x->inte.low, x->inte.high, x->max,
                rb_max_depth(x), rb_min_depth(x));
        rb_show(x->right);
    }
}

int rb_tree::rb_max_depth(typename rb_tree::prb_type x) {
    if (x == nil)
        return 0;
    int l = rb_max_depth(x->left);
    int r = rb_max_depth(x->right);
    return (l > r ? l : r) + 1;
}

int rb_tree::rb_min_depth(typename rb_tree::prb_type x) {
    if (x == nil)
        return 0;
    int l = rb_min_depth(x->left);
    int r = rb_min_depth(x->right);
    return (l < r ? l : r) + 1;
}
