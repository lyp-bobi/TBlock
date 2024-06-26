//
// Created by Chuang on 2021/6/2.
//

#ifndef TBLOCK_TRAJECTORY_HPP
#define TBLOCK_TRAJECTORY_HPP
#include "Point.hpp"
#include <vector>
#include <string>
#include "MBR.hpp"
#include "chrono"
#include <memory>
#include "postgis.h"
extern double timer_traj;


enum BType{
    T_box1 = 0,
    T_box2,
    T_block1,
    T_block2,
    T_end
};

struct BEnable{
    bool enable[T_end]={false, false, false, false};
};

struct Point_c{
    double x,y,z;
};

struct TBlock_c{
public:
    int m_size;
    BType *m_type;

};

struct TBOX_SERL{
    uint32_t varsize;
    float coords[0];
};

struct TBLOCK_SERL{
    uint32_t varsize;
    char data[0];
};

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
    Trajectory(POSTGIS_POINTARRAY* ps);
    Trajectory(std::vector<Point>& in)
    {
        m_points=in;
    }
    void loadFromString(std::string s, char pnt_sep = ',', char dim_sep = ' ');

    POSTGIS_POINTARRAY *asptarray();

    void resample(int numseg);

    Point operator[](IntRange r);

    MBR getMBR();

    int locateTime(int l,int h, double time, int dir);
    bool stepIn(MBR &r);
    bool passBy(MBR &r);

    Trajectory toUV();
    Trajectory toXY();
};
#endif //TBLOCK_TRAJECTORY_HPP
