//
// Created by Chuang on 2021/10/5.
//
#include "TBlockKey.h"
#include "Trajectory.h"
#include "vector"
using std::vector;
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



double area(Point &a, Point &b, int type){
    if(type==1){
        return (fabs(b.m_x-a.m_x))*(fabs(b.m_y-a.m_y));
    }else if(type == 2){
        double du = (fabs(b.m_x-a.m_x) + fabs(b.m_y-a.m_y))/sqrt(2),
                dv = (fabs(b.m_x-a.m_x) - fabs(b.m_y-a.m_y))/sqrt(2);
        return (du)*(dv);
    }
    return 1e300;
}

/*
 * 0 = not Tblock
 * 1 = Tblock
 * 2 = Tblockrot
 */
int blockType(Trajectory &tj, int a, int b)
{
    if(a>b){
        int tmp;
        tmp=a;
        a=b;
        b=tmp;
    }
    double area1 = NAN, area2 = NAN;

    //block1
    for(int i=a;i<=b;i++){
        if(!inblock1(tj.m_points[a],tj.m_points[b],tj.m_points[i]))
            break;
        if(i==b)
            area1 = area(tj.m_points[a], tj.m_points[b], 1);
    }
    for(int i=a;i<=b;i++){
        if(!inblock2(tj.m_points[a],tj.m_points[b],tj.m_points[i]))
            break;
        if(i==b)
            area2 =area1 = area(tj.m_points[a], tj.m_points[b], 2);
    }
    if(area1 == area2 && area1 == NAN) return 0;
    if(area2 == NAN || area1 <= area2) return 1;
    if(area1 == NAN || area1 > area2) return 2;
    return 0;
}

vector<double> OPTcost(Trajectory &tj)
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
    for(i=1;i<tj.m_points.size();i++) {// last id of points
        for(k=1;k<=i;k++) { // num of boxes
            if(k==1){
                j=0;
                dmat[i][k] = area(tj.m_points[i], tj.m_points[j],
                                  blockType(tj, j, i));
            }
            else {
                for (j = 0; j < i; j++) { //last chosen point
                    if(j >= k - 1) {
                        double value = dmat[j][k - 1] +
                                       area(tj.m_points[i], tj.m_points[j],
                                            blockType(tj, j, i));
                        dmat[i][k] = std::min(dmat[i][k], value);
                    }
                }
            }
        }
    }
    return dmat.back();
}


TBlockKey OPTBlock(Trajectory &tj, int nbox)
{
    /*
     * DP table
     * |the first point| using one box| using two box | ...
     * |the second point| using one box| using two box | ...
     */
    vector<vector<double>> dmat; // cost
    vector<vector<int>> dtype; // the block type of previous block
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
    for(i=1;i<tj.m_points.size();i++) {// num of point
        for(k=1;k<=std::min(i,nbox);k++) { // num of boxes
            if(k==1){
                j=0;
                dmat[i][k] = area(tj.m_points[i], tj.m_points[j],
                                  blockType(tj, j, i));
            }
            for (j = 0; j < i; j++) { //last point
                int type = blockType(tj, j, i);
                double value = dmat[i][k-1] + area(tj.m_points[i], tj.m_points[j],
                                                   type);
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


// deprecated : loss type, we use size type now
//std::shared_ptr<TBlockKey> OPTBlock(Trajectory &tj){
//    double* dcost = new double[tj.m_points.size() + 1];
//    double* dprev = new double[tj.m_points.size() + 1];
//    int* dtype = new int[tj.m_points.size() + 1];
//    dcost[0]=0;
//    dcost[1] = loss(tj.m_points[0], tj.m_points[1], 1);
//    dprev[0] = 0;
//    dprev[1] = 0;
//    dtype[0] =0;
//    dtype[1] = 1;
//    for(int i=2;i<tj.m_points.size();i++)
//    {
//        dcost[i] = 1e10;
//        for(int j=0;j<i;j++){
//            int type = blockType(tj, j, i);
//            if(type>0)
//            {
//                double c = loss(tj.m_points[j], tj.m_points[i], type);
//                if(dcost[j] + c < dcost[i]) {
//                    dcost[i] = dcost[j] + c;
//                    dprev[i] = j;
//                    dtype[i] = type;
//                }
//            }
//        }
//    }
//    //form
//    TBlockKey *res = new TBlockKey;
//    vector<int> rev;
//    int cur = tj.m_points.size();
//    while(cur!=0){
//        rev.emplace_back(cur);
//        cur=dprev[cur];
//    }
//    rev.emplace_back(0);
//    res->m_size = rev.size()-1;
//    for(int i = rev.size();i>=0;i--){
//        res->m_type.emplace_back( dtype[rev[i]]);
//        res->m_pts.emplace_back(tj.m_points[rev[i]]);
//    }
//    delete[] dtype;
//    delete[] dprev;
//    delete[] dcost;
//    return std::shared_ptr<TBlockKey>(res);
//}