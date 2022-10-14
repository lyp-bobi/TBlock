//
// Created by Chuang on 2021/6/3.
//
#ifndef TBLOCK_MBR_HPP
#define TBLOCK_MBR_HPP
#include "Point.hpp"
class MBR{
public:
    double m_xmin,m_xmax,m_ymin,m_ymax;
    double m_tmin, m_tmax;
    MBR(double xmin, double xmax, double ymin, double ymax, double tmin, double tmax)
    : m_xmin(xmin), m_xmax(xmax), m_ymin(ymin), m_ymax(ymax),m_tmin(tmin), m_tmax(tmax){};
    void combinePoint(Point &p);

    bool contains(Point &p);

    bool intersects(MBR &r);
    bool intersects(Point &s, Point &e);
};

//bool intersects(GEOSGeometry &g,double ts, double te, Point &s, Point&e);



#endif //TBLOCK_MBR_HPP
