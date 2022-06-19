//
// Created by Chuang on 2021/10/6.
//

#ifndef TBLOCK_TBLOCKKEY_H
#define TBLOCK_TBLOCKKEY_H
#include <vector>
#include <sstream>
#include "Trajectory.h"



class TBlockKey{
public:
    int m_size;
    std::vector<int> m_type;
    std::vector<Point> m_pts;
};


enum BType{
    T_box1 = 0,
    T_box2,
    T_block1,
    T_block2,
    T_end
};


enum TBlockLossType{
    LOSS_AREA,
    LOSS_VOLUME,
    LOSS_QUERY
};

struct TBlockRouteEntry{
    int m_ps;
    int m_pe;
    BType m_type;
    TBlockRouteEntry(int ps, int pe, BType type)
    :m_ps(ps), m_pe(pe), m_type(type)
    {}
//    TBlockRouteEntry& operator=(TBlockRouteEntry& r)
//    {
//        m_ps = r.m_ps;
//        m_pe = r.m_pe;
//        m_type = r. m_type;
//    }
};

struct TBlockRoute{
    std::vector<TBlockRouteEntry> m_route;
    double cost;
public:
    std::string toString()
    {
        std::ostringstream ss;
        for(auto &e: m_route)
        {
            ss<<e.m_ps<<","<<e.m_pe<<","<<e.m_type<<"\t";
        }
        ss<< cost;
        return ss.str();
    }
};

struct BEnable{
    bool enable[T_end]={false, false, false, false};
};

class BSize{
public:
    double size[T_end]={1e300,1e300,1e300,1e300};
};


extern TBlockKey OPTBlock(Trajectory &tj, int nbox);

extern std::vector<TBlockRoute> OPTcost(Trajectory &tj, BEnable ena);

extern std::vector<double> OPTcostGlobal(std::vector<Trajectory> &tjs, int nbox, BEnable ena);

#endif //TBLOCK_TBLOCKKEY_H
