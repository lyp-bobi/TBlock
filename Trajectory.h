//
// Created by Chuang on 2021/6/2.
//

#ifndef TBLOCK_TRAJECTORY_H
#define TBLOCK_TRAJECTORY_H
#include "Point.h"
#include <vector>
#include <string>
#include "MBR.h"
#include "chrono"
extern double timer_traj;

class Trajectory{
public:
    std::vector<Point> m_points;

    std::string toString() const ;
    void loadFromString(std::string s);
    MBR getMBR();

    int locateTime(int l,int h, double time, int dir);
    bool stepIn(MBR &r);
    bool passBy(MBR &r);
};
#endif //TBLOCK_TRAJECTORY_H
