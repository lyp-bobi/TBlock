//
// Created by Chuang on 2021/10/6.
//

#ifndef TBLOCK_TBLOCKKEY_HPP
#define TBLOCK_TBLOCKKEY_HPP
#include <vector>
#include <sstream>
#include "Trajectory.hpp"

class TBlockKey{
public:
    int m_size;
    std::vector<int> m_type;
    std::vector<Point> m_pts;
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
    double cost = 1e300;
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

struct SingleBound {
    double xmin, xmax,ymin, ymax;
};

#define BoundSize(b) (((b).xmax - (b).xmin) * ((b).ymax - (b).ymin))

struct BoundPreCalc{
    std::vector<std::vector<SingleBound>> boundxy;
    std::vector<std::vector<SingleBound>> bounduv;
    std::vector<std::vector<BSize>> volume;
    std::vector<std::vector<bool>> availxy;
    std::vector<std::vector<bool>> availuv;
};

extern BoundPreCalc getBoundMat(Trajectory &tj);



//extern TBlockKey OPTBlock(Trajectory &tj, int nbox, BEnable ena);

extern std::vector<TBlockRoute> OPTcost(Trajectory &tj, BEnable ena,int numbox = 1e9);

extern TBlockRoute OPTcostMin(Trajectory &tj, BEnable ena);

extern std::vector<double> OPTcostGlobal(std::vector<Trajectory> &tjs, int nbox, BEnable ena);

TBlockRoute GreedyPath(Trajectory &tj, BEnable ena);

TBlockRoute GreedyPathMod(Trajectory &tj, BEnable ena, int numseg);

TBlockRoute GreedyBox(Trajectory &tj, BEnable ena, int numseg);

#endif //TBLOCK_TBLOCKKEY_HPP
