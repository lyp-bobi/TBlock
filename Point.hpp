//
// Created by Chuang on 2021/6/2.
//

#ifndef TBLOCK_POINT_HPP
#define TBLOCK_POINT_HPP
#include <stdint.h>
#include <iostream>
#include <cmath>
//#include <geos_c.h>

class Point
{
    public:
    Point(){};
    Point(double x, double y, double t);
    Point(const Point& p);
    virtual ~Point();

    virtual Point& operator=(const Point& p);
    virtual bool operator==(const Point& p) const;

    Point operator+ (const Point &b) const;
    Point operator- (const Point &b) const;

//    bool inside(GEOSGeometry &g) const;

    //
    // ISerializable interface
    //
    virtual uint32_t getByteArraySize() const;

    double getDistance(Point &p)
    {
        return std::sqrt((m_x-p.m_x)*(m_x-p.m_x) + (m_y-p.m_y)*(m_y-p.m_y));
    }

    public:
    double m_x,m_y;
    double m_t;
    inline double m_pCoords(int i) const{
        if(i==0) return m_x;
        if(i==1) return m_y;
        if(i==2) return m_t;
        return 0;
    }
    static Point makemid(const Point &tp1, const Point &tp2, double t);
    friend std::ostream& operator<<(std::ostream& os, const Point& pt);
}; // Point

std::ostream& operator<<(std::ostream& os, const Point& pt);

#define doubleAreaTriangle(pax,pay,pbx,pby,pcx,pcy) (((pbx - pax) * (pcy - pay)) - ((pcx - pax) * (pby - pay)))

#define collinear(a,b,c)  (doubleAreaTriangle(a.m_x,a.m_y,b.m_x,b.m_y,c.m_x,c.m_y)==0)

#define leftOf(a,b,c)  (doubleAreaTriangle(a.m_x,a.m_y,b.m_x,b.m_y,c.m_x,c.m_y)>0)

static inline bool intersectsProper(const Point &a, const Point &b, const Point &c, const Point &d) {
    if ( collinear(a, b, c) || collinear(a, b, d) ||
         collinear(c, d, a) || collinear(c, d, b)) {
        return false;
    }
    return ((leftOf(a, b, c) ^ leftOf(a, b, d)) &&
            (leftOf(c, d, a) ^ leftOf(c, d, b)));
}

#define valueBetween(a,b,c)   (((a <= c) && (c <= b)) || ((a >= c) && (c >= b)) )

static inline bool between(const Point &a, const Point &b, const Point &c) {
    if ( !collinear(a, b, c) ) {
        return false;
    }
    if ( a.m_x != b.m_x ) { // a & b are not on the same vertical, compare on x axis
        return valueBetween(a.m_x,b.m_x,c.m_x);
    } else { // a & b are a vertical segment, we need to compare on y axis
        return valueBetween(a.m_y,b.m_y,c.m_y);
    }
}

static inline bool lineIntersects(const Point &l1s, const Point &l1e, const Point &l2s, const Point &l2e) {
    if (intersectsProper(l1s, l1e, l2s, l2e)) {
        return true;
    }
    else
        return between(l1s, l1e, l2s) || between(l1s, l1e, l2e) ||
               between(l2s, l2e, l1s) || between(l2s, l2e, l1e);
}

static inline Point makemid(const Point &p1, const Point &p2, double t){
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

#endif //TBLOCK_POINT_HPP
