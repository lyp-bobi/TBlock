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


extern TBlockKey OPTBlock(Trajectory &tj, int nbox);

extern std::vector<double> OPTcost(Trajectory &tj);

extern std::vector<double> OPTcostGlobal(std::vector<Trajectory> &tjs, int nbox);

#endif //TBLOCK_TBLOCKKEY_H
