//
// Created by Chuang on 2021/10/5.
//
#include "TBlockKey.h"
#include "Trajectory.h"
#include "vector"
using std::vector;



/*is p in block1(a,b)*/
inline bool inblock1(Point &a, Point &b, Point &p){
    return ((p.m_x <= a.m_x && p.m_x >= b.m_x) ||
            (p.m_x >= a.m_x && p.m_x <= b.m_x))
           && ((p.m_y <= a.m_y && p.m_y >= b.m_y) ||
               (p.m_y >= a.m_y && p.m_y <= b.m_y));
}

inline bool inblock2(Point &a, Point &b, Point &p){
    Point ra(a.m_x+a.m_y, a.m_x-a.m_y,a.m_t),
            rb(b.m_x+b.m_y, b.m_x-b.m_y,b.m_t),
            rp(p.m_x+p.m_y, p.m_x-p.m_y,p.m_t);
    return inblock1(ra,rb,rp);
}


double areaBlock(Point &a, Point &b, BType type){
    if(type==T_block1){
        return (fabs(b.m_x-a.m_x))*(fabs(b.m_y-a.m_y));
    }else if(type == T_block2){
        double du = fabs((b.m_x + b.m_y -(a.m_x+a.m_y))),
                dv = fabs((b.m_x - b.m_y- (a.m_x-a.m_y)));// /sqrt(2)
        return (du)*(dv)/2;
    }
    return 1e300;
}

BType minType(BSize size){
    BType res = T_box1;
    for(auto &t:{T_box2, T_block1, T_block2})
    {
        if(size.size[t]< size.size[res])
            res = t;
    }
    return res;
}

double minSize(BSize size){
    BType res = T_box1;
    for(auto &t:{T_box2, T_block1, T_block2})
    {
        if(size.size[t]< size.size[res])
            res = t;
    }
    return size.size[res];
}

BSize blockSize(Trajectory &tj, int a, int b, BEnable ena)
{
    if(a>b){
        int tmp;
        tmp=a;
        a=b;
        b=tmp;
    }
    BSize size = {1e300,1e300, 1e300,1e300};

    //box1
    double xmin=0,xmax=0, ymin=0, ymax=0;
    if(ena.enable[T_box1]) {
        xmin = xmax = tj.m_points[a].m_x;
        ymin = ymax = tj.m_points[a].m_y;
        for (int i = a+1; i <= b; i++) {
            if(xmin > tj.m_points[i].m_x)
                xmin = tj.m_points[i].m_x;
            if(xmax < tj.m_points[i].m_x)
                xmax = tj.m_points[i].m_x;
            if(ymin > tj.m_points[i].m_y)
                ymin = tj.m_points[i].m_y;
            if(ymax < tj.m_points[i].m_y)
                ymax = tj.m_points[i].m_y;
        }
        size.size[T_box1] = (xmax - xmin)*(ymax-ymin);
    }


    // box2
    double umin=0,umax=0, vmin=0, vmax=0;
    if(ena.enable[T_box2]) {
        umin = umax = tj.m_points[a].m_x + tj.m_points[a].m_y;
        vmin = vmax = tj.m_points[a].m_x - tj.m_points[a].m_y;
        for (int i = a+1; i <= b; i++) {
            double u = tj.m_points[i].m_x + tj.m_points[i].m_y;
            double v = tj.m_points[i].m_x - tj.m_points[i].m_y;
            if(umin > u)
                umin = u;
            if(umax < u)
                umax = u;
            if(vmin > v)
                vmin = v;
            if(vmax < v)
                vmax = v;
        }
        size.size[T_box2] = (umax - umin)*(vmax-vmin)/2;
    }


    //block1
    if(ena.enable[T_block1]) {
        for (int i = a; i <= b; i++) {
            if (!inblock1(tj.m_points[a], tj.m_points[b], tj.m_points[i]))
                break;
            if (i == b) {
                size.size[T_block1] = areaBlock(tj.m_points[a], tj.m_points[b], T_block1);
            }
        }
    }

    //block2
    if(ena.enable[T_block2]) {
        for (int i = a; i <= b; i++) {
            if (!inblock2(tj.m_points[a], tj.m_points[b], tj.m_points[i]))
                break;
            if (i == b) {
                size.size[T_block2] = areaBlock(tj.m_points[a], tj.m_points[b], T_block2);
            }
        }
    }

    return size;
}

vector<double> OPTcost(Trajectory &tj, BEnable ena)
{
    /*
     * DP table
     * |the first point| using one box| using two box | ...
     * |the second point| using one box| using two box | ...
     */
    vector<vector<double>> dmat;
    dmat.resize(tj.m_points.size());
    int i=1, j, k;
    for(auto &vec: dmat)
    {
        vec.resize(i); /*n+1 points can at most be expressed by n boxes*/
        for(auto &d:vec)
        {
            d = 1e300;
        }
        i++;
    }
    BSize s = {1e300,1e300, 1e300,1e300};;
    for(i=1;i<tj.m_points.size();i++) {// last id of points
        for(k=1;k<=i;k++) { // num of boxes
            if(k==1){
                j=0;
                s = blockSize(tj,i,j,ena);
                dmat[i][k] = minSize(s);
            }
            else {
                for (j = 0; j < i; j++) { //last chosen point
                    if(j >= k - 1) {
                        s = blockSize(tj,i,j,ena);
                        double value = dmat[j][k - 1] + minSize(s);
                        dmat[i][k] = std::min(dmat[i][k], value);
                    }
                }
            }
        }
    }
    return dmat.back();
}


TBlockKey OPTBlock(Trajectory &tj, int nbox, BEnable ena)
{
    /*
     * DP table
     * |the first point| using one box| using two box | ...
     * |the second point| using one box| using two box | ...
     */
    vector<vector<double>> dmat; // cost
    vector<vector<BType>> dtype; // the block type of previous block
    vector<vector<int>> dprev; // the index of previous block
    dmat.resize(tj.m_points.size());
    dtype.resize(tj.m_points.size());
    dprev.resize(tj.m_points.size());
    int i=0, j, k;
    for(auto &vec: dmat)
    {
        vec.resize(std::min(i, nbox));
        for(auto &d:vec)
        {
            d = 1e300;
        }
        i++;
    }
    BSize s = {1e300,1e300, 1e300,1e300};;
    for(i=1;i<tj.m_points.size();i++) {// num of point
        for(k=1;k<=std::min(i,nbox);k++) { // num of boxes
            if(k==1){
                j=0;
                s = blockSize(tj,i,j,ena);
                dmat[i][k] = minSize(s);
            }
            for (j = 0; j < i; j++) { //last point
                s = blockSize(tj,i,j,ena);
                BType type = minType(s);
                double value = dmat[i][k-1] + minSize(s);
                if(dmat[i][k] > value) {
                    dmat[i][k] = value;
                    dprev[i][k] = j;
                    dtype[i][k] = type;
                }
            }
        }
    }
    TBlockKey res;
    vector<int> rev;
    int cur = tj.m_points.size();
    int curbox = nbox;
    while(cur!=0){
        rev.emplace_back(cur);
        cur=dprev[cur][curbox-1];
    }
    rev.emplace_back(0);
    res.m_size = rev.size()-1;
    for(int i = rev.size();i>=0;i--){
        res.m_type.emplace_back( dtype[rev[i]][i]);
        res.m_pts.emplace_back(tj.m_points[rev[i]]);
    }
    return res;
}

