//
// Created by Chuang on 2021/6/2.
//


#include <cstring>
#include <limits>
#include "Point.h"


Point::Point(double x, double y, double t) {
    m_x=x;
    m_y=y;
    m_t=t;
}


Point::Point(const Point& p)
        :m_x(p.m_x),m_y(p.m_y), m_t(p.m_t)
{
}

Point::~Point()
{
}

Point& Point::operator=(const Point& p)
{
    if (this != &p)
    {
        m_x = p.m_x;
        m_y = p.m_y;
        m_t = p.m_t;
    }

    return *this;
}

bool Point::operator==(const Point& p) const
{
    return !(m_t < p.m_t - std::numeric_limits<double>::epsilon() ||
             m_t > p.m_t + std::numeric_limits<double>::epsilon() ||
             m_x < p.m_x - std::numeric_limits<double>::epsilon() ||
             m_x > p.m_x + std::numeric_limits<double>::epsilon() ||
             m_y < p.m_y - std::numeric_limits<double>::epsilon() ||
             m_y > p.m_y + std::numeric_limits<double>::epsilon());

}

Point Point::operator+(const Point &b) const {
    Point res;
    res.m_x = m_x + b.m_x;
    res.m_y = m_y + b.m_y;
    res.m_t = m_t + b.m_t;
    return res;
}

Point Point::operator-(const Point &b) const {
    Point res;
    res.m_x = m_x - b.m_x;
    res.m_y = m_y - b.m_y;
    res.m_t = m_t - b.m_t;
    return res;
}

//
// ISerializable interface
//
uint32_t Point::getByteArraySize() const
{
    return 3* sizeof(double);
}



std::ostream& operator<<(std::ostream& os, const Point& pt)
{
    uint32_t i;

    os<<pt.m_x<<","<<pt.m_y << "," << pt.m_t;

    return os;
}

Point Point::makemid(const Point &p1, const Point &p2, double t){
    if(p1.m_t==p2.m_t)
        return Point(p1);
    double p1c[2];
    double p2c[2];
    double t1=p1.m_t;
    double t2=p2.m_t;
    p1c[0]=p1.m_x;
    p2c[0]=p2.m_x;
    p1c[1]=p1.m_y;
    p2c[1]=p2.m_y;
    double h1= (t-t1)/(t2-t1);
    double h2= (t2-t)/(t2-t1);
    double p3c[2];
    for(int i=0;i<2;i++){
        p3c[i]=h2*p1c[i]+h1*p2c[i];
    }
    auto res=Point(p3c[0], p3c[1], t);
    return res;
}

//bool Point::inside(GEOSGeometry &g) const {
//    GEOSGeometry *p = GEOSGeom_createPointFromXY(m_x, m_y);
//    bool ret = GEOSIntersects(&g,p);
//    free(p);
//    return ret;
//}