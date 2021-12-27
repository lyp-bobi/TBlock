//
// Created by Chuang on 2021/6/2.
//

#ifndef TBLOCK_TBLOCKPROTO_H
#define TBLOCK_TBLOCKPROTO_H

#include "Trajectory.h"
extern double timer_tblock;

class  TBlockProto{
public:
    std::vector<Point> m_points;
    uint32_t m_tag = 0;
    void cvt_greedy(Trajectory &traj);

    int locateTime(int l,int h, double time, int dir);

    bool stepIn(MBR &r);
    bool passBy(MBR &r);
};

extern int isTBlock(Trajectory &tj, int s, int e);

#endif //TBLOCK_TBLOCKPROTO_H
