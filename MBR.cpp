//
// Created by Chuang on 2021/6/3.
//

#include "MBR.hpp"

void MBR::combinePoint(Point &p) {
    m_xmin = std::min(m_xmin, p.m_x);
    m_xmax = std::max(m_xmax, p.m_x);
    m_ymin = std::min(m_ymin, p.m_y);
    m_ymax = std::max(m_ymax, p.m_y);
    m_tmin = std::min(m_tmin, p.m_t);
    m_tmax = std::max(m_tmax, p.m_t);
}

bool MBR::intersects(MBR &r)
{
    if(m_xmin>r.m_xmax || m_xmax<r.m_xmin) return false;
    if(m_ymin>r.m_ymax || m_ymax<r.m_ymin) return false;
    if(m_tmin>r.m_tmax || m_tmax<r.m_tmin) return false;
    return true;
}

bool MBR::contains(Point& p)
{
    return (p.m_x>=m_xmin
                && p.m_x<=m_xmax
                && p.m_y>=m_ymin
                && p.m_y<=m_ymax
                &&p.m_t>=m_tmin
                && p.m_t<=m_tmax);
}

bool MBR::intersects(Point &s, Point &e) {
    if(e.m_t<m_tmin||s.m_t>m_tmax) return false;
    Point ll = Point(m_xmin, m_ymin, m_tmin);
    Point ur = Point(m_xmax, m_ymax, m_tmax);
    Point ul = Point(m_xmax, m_ymin, m_tmin);
    Point lr = Point(m_xmin, m_ymax, m_tmin);

    Point ss, ee;
    if(s.m_t<m_tmin){
        ss = makemid(s,e, m_tmin);
    }else{
        ss = s;
    }
    if(e.m_t>m_tmax){
        ee = makemid(s,e, m_tmax);
    }else{
        ee = e;
    }

    //Check whether either or both the endpoints are within the region OR
    //whether any of the bounding segments of the Region intersect the segment
    return (contains(ss) || contains(ee) ||
            lineIntersects(ss,ee,ll, ul) || lineIntersects(ss,ee,ul, ur) ||
            lineIntersects(ss,ee,ur, lr) || lineIntersects(ss,ee,lr, ll));
}

//bool intersects(GEOSGeometry &g,double m_tmin, double m_tmax, Point &s, Point&e){
//    if(e.m_t<m_tmin||s.m_t>m_tmax) return false;
//
//    Point ss, ee;
//    if(s.m_t<m_tmin){
//        ss = makemid(s,e, m_tmin);
//    }else{
//        ss = s;
//    }
//    if(e.m_t>m_tmax){
//        ee = makemid(s,e, m_tmax);
//    }else{
//        ee = e;
//    }
//    GEOSCoordSequence* seq = GEOSCoordSeq_create(2,2);
//    GEOSCoordSeq_setXY(seq,0,ss.m_x, ss.m_y);
//    GEOSCoordSeq_setXY(seq,1,ee.m_x, ee.m_y);
//    GEOSGeometry *line = GEOSGeom_createLineString(seq);
//    return GEOSIntersects(&g,line);
//}