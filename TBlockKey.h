//
// Created by Chuang on 2021/10/6.
//

#ifndef TBLOCK_TBLOCKKEY_H
#define TBLOCK_TBLOCKKEY_H
#include <vector>
#include "Trajectory.h"



class TBlockKey{
public:
    int m_size;
    std::vector<int> m_type;
    std::vector<Point> m_pts;
};

enum TBlockLossType{
    LOSS_AREA,
    LOSS_VOLUME,
    LOSS_QUERY
};

enum BType{
    T_box1 = 0,
    T_box2,
    T_block1,
    T_block2,
    T_end
};

struct BEnable{
    bool enable[T_end]={false,false,false,false};
};

class BSize{
public:
    double size[T_end]={1e300,1e300,1e300,1e300};
};


extern TBlockKey OPTBlock(Trajectory &tj, int nbox);

extern std::vector<double> OPTcost(Trajectory &tj, BEnable ena);

extern std::vector<double> OPTcostGlobal(std::vector<Trajectory> &tjs, int nbox, BEnable ena);

#endif //TBLOCK_TBLOCKKEY_H
