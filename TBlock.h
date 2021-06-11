//
// Created by Chuang on 2021/6/2.
//

#ifndef TBLOCK_TBLOCK_H
#define TBLOCK_TBLOCK_H

#include "Trajectory.h"
extern double timer_tblock;

class TBlock{
public:
    std::vector<Point> m_points;
    uint32_t m_tag = 0;
    void cvt_greedy(Trajectory &traj);

    int locateTime(int l,int h, double time, int dir);

    bool stepIn(MBR &r);
    bool passBy(MBR &r);
};

#endif //TBLOCK_TBLOCK_H
