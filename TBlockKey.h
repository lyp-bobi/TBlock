//
// Created by Chuang on 2021/10/6.
//

#ifndef TBLOCK_TBLOCKKEY_H
#define TBLOCK_TBLOCKKEY_H
#include <vector>
#include <sstream>
#include "Trajectory.h"

#include <TBlock_c.h>


class TBlockKey{
public:
    int m_size;
    std::vector<int> m_type;
    std::vector<Point> m_pts;
};

struct IntRange{
    int m_plast;
    double m_ratio = 0;
    IntRange(int i):m_plast(i),m_ratio(0){};
};

class BSize{
public:
    double size[T_end]={1e300,1e300,1e300,1e300};
};

enum TBlockLossType{
    LOSS_AREA,
    LOSS_VOLUME,
    LOSS_QUERY
};

struct TBlockRouteEntry{
    IntRange m_ps;
    IntRange m_pe;
    BType m_type;
    BSize m_size;

    TBlockRouteEntry(IntRange ps, IntRange pe, BType type, BSize s)
    :m_ps(ps), m_pe(pe), m_type(type), m_size(s)
    {}

    struct TBRE_cmp_size{
        bool operator()(TBlockRouteEntry &a, TBlockRouteEntry &b)
        {
            return a.m_size.size[a.m_type]< b.m_size.size[b.m_type];
        }
    };
    struct TBRE_cmp_ps{
        bool operator()(TBlockRouteEntry &a, TBlockRouteEntry &b)
        {
            if(a.m_ps.m_plast==b.m_ps.m_plast) return a.m_ps.m_ratio > b.m_ps.m_ratio;
            return a.m_ps.m_plast > b.m_ps.m_plast;
        }
    };
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
            ss<<e.m_ps.m_plast<<","<<e.m_pe.m_plast<<","<<e.m_type<<"\t";
        }
        ss<< cost;
        return ss.str();
    }
};




extern TBlockKey OPTBlock(Trajectory &tj, int nbox, BEnable ena);

extern std::vector<TBlockRoute> OPTcost(Trajectory &tj, BEnable ena,int numbox = 1e9);

extern std::vector<double> OPTcostGlobal(std::vector<Trajectory> &tjs, int nbox, BEnable ena);

TBlockRoute GreedyPath(Trajectory &tj, BEnable ena);

TBlockRoute GreedyPathElite(Trajectory &tj, BEnable ena, int numseg);

#endif //TBLOCK_TBLOCKKEY_H
