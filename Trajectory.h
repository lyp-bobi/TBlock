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
#include <memory>
#include <postgis.h>
extern double timer_traj;

struct IntRange{
    int m_plast;
    double m_ratio = 0;
    IntRange(int i):m_plast(i),m_ratio(0){};
};

class Trajectory{
public:
    std::vector<Point> m_points;

    std::string toString() const ;
    Trajectory(){};
    Trajectory(std::vector<Point>& in)
    {
        m_points=in;
    }
    void loadFromString(std::string s);

    POINTARRAY *asptarray();

    Point operator[](IntRange r);

    MBR getMBR();

    int locateTime(int l,int h, double time, int dir);
    bool stepIn(MBR &r);
    bool passBy(MBR &r);
};
#endif //TBLOCK_TRAJECTORY_H
