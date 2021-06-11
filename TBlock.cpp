//
// Created by Chuang on 2021/6/2.
//

#include "TBlock.h"
double timer_tblock = 0;

using std::vector;
using std::pair;

void TBlock::cvt_greedy(Trajectory &traj) {
    double x, y;
    m_points.clear();
    m_points.reserve(traj.m_points.size());
    int directionx = 0, directiony = 0; //-1 is decrease, 0 is unknown, 1 is increase
    for (int i = 0; i < traj.m_points.size(); i++) {
        if (i == 0) {
            x = traj.m_points[i].m_x;
            y = traj.m_points[i].m_y;
            m_points.emplace_back(traj.m_points[i]);
            directionx = 0;
            directiony = 0;
            continue;
        }
        if (
                (directionx == 1 && traj.m_points[i].m_x < x)
                || (directionx == -1 && traj.m_points[i].m_x > x)
                || (directiony == 1 && traj.m_points[i].m_y < y)
                || (directiony == -1 && traj.m_points[i].m_y > y)
                ) {
            m_points.emplace_back(traj.m_points[i - 1]);
            x = traj.m_points[i - 1].m_x;
            y = traj.m_points[i - 1].m_y;
            directionx = 0;
            directiony = 0;
        }
        if (directionx == 0) {
            if (traj.m_points[i].m_x > x) directionx = 1;
            else if (traj.m_points[i].m_x < x) directionx = -1;
        }
        if (directiony == 0) {
            if (traj.m_points[i].m_y > y) directiony = 1;
            else if (traj.m_points[i].m_y < y) directiony = -1;
        }
        x = traj.m_points[i].m_x;
        y = traj.m_points[i].m_y;
        if (i == traj.m_points.size() - 1) {
            m_points.emplace_back(traj.m_points[i]);
        }
    }
    m_tag = m_points.size()-1;
    int vpcur = 0;
    for (int i = 0; i < traj.m_points.size(); i++) {
        if (traj.m_points[i].m_t == m_points[vpcur].m_t) {
            ++vpcur;
            continue;
        }
        m_points.emplace_back(traj.m_points[i]);
    }
}


inline int TBlock::locateTime(int l, int h, double time, int dir) {
    int m,ll = l, hh =h;
    while (l < h - 1) {
        m = (l+h)/2;
        if (m_points[m].m_t == time) {
            return m;
        } else if (m_points[m].m_t > time) {
            h = m;
        } else {
            l = m;
        }
    }
    if(l==ll&&m_points[ll].m_t>time) return ll;
    if(h==hh&&m_points[hh].m_t<time) return hh;
    if (dir < 0)return l;
    else return l + 1;
}

inline int checkplace(double p, double l, double h) {
    if (p < l) return -1;
    if (p > h) return 1;
    return 0;
}

bool TBlock::stepIn(MBR &r) {
    int l, h, ll, hh, cursor1, cursor2;
    l = locateTime(0, m_tag, r.m_tmin, -1);
    h = locateTime(l, m_tag, r.m_tmax, 1);
    ll = locateTime(m_tag+1, m_points.size(), r.m_tmin, 1);
    hh = locateTime(std::max((int)m_tag+1,ll-1), m_points.size(), r.m_tmax, -1);
    int xin_last, yin_last, xin_this, yin_this;
    xin_last = checkplace(m_points[l].m_x, r.m_xmin, r.m_xmax);
    yin_last = checkplace(m_points[l].m_y, r.m_ymin, r.m_ymax);
    if (xin_last == 0 && yin_last==0
        && m_points[l].m_t>=r.m_tmin && m_points[l].m_t<=r.m_tmax)
        return true;
    cursor1 = l;
    cursor2 = ll;
    while(cursor2<=hh){
        cursor1++;
        xin_this = checkplace(m_points[cursor1].m_x, r.m_xmin, r.m_xmax);
        yin_this = checkplace(m_points[cursor1].m_y, r.m_ymin, r.m_ymax);
        if(xin_this == 0 && yin_this ==0 &&
                (cursor1<h || (m_points[cursor1].m_t>=r.m_tmin && m_points[cursor1].m_t<=r.m_tmax)))
            return true;
        if (abs(xin_this + xin_last) >= 2 || abs(yin_this + yin_last) >= 2) {
            double c1t = m_points[cursor1].m_t;
            while(c1t>m_points[cursor2].m_t&&cursor2<=hh){
                cursor2++;
            }
        }else{
            while(m_points[cursor1].m_t>m_points[cursor2].m_t && cursor2<=hh){
                if (m_points[cursor2].m_x >= r.m_xmin && m_points[cursor2].m_x <= r.m_xmax
                    && m_points[cursor2].m_y >= r.m_ymin && m_points[cursor2].m_y <= r.m_ymax)
                    return true;
                cursor2++;
            }
        }
        yin_last = yin_this;
        xin_last = xin_this;
    }
    for(; cursor1 <= h; cursor1++){
        if (m_points[cursor1].m_x >= r.m_xmin && m_points[cursor1].m_x <= r.m_xmax
            && m_points[cursor1].m_y >= r.m_ymin && m_points[cursor1].m_y <= r.m_ymax
            && ((cursor1>l && cursor1 < h) || (m_points[cursor1].m_t >= r.m_tmin && m_points[cursor1].m_t <= r.m_tmax)))
            return true;
    }
    return false;


    /*
    if (r.m_tmax < m_points.front().m_t || r.m_tmin > m_points[m_tag].m_t) {
        return false;
    }
    int l = locateTime(0, m_tag, r.m_tmin, -1),
            h = locateTime(l, m_tag, r.m_tmax, 1);
    int xin_last, yin_last, xin_this, yin_this;
    xin_last = checkplace(m_points[l].m_x, r.m_xmin, r.m_xmax);
    yin_last = checkplace(m_points[l].m_y, r.m_ymin, r.m_ymax);
    if (xin_last == 0 && yin_last==0
        && m_points[l].m_t>=r.m_tmin && m_points[l].m_t<=r.m_tmax)
        return true;
    int searchl = l;
    vector<pair<int, int>> intervals;
    bool searching_flag = false;
    for (int i = l + 1; i <= h; i++) {
        xin_this = checkplace(m_points[i].m_x, r.m_xmin, r.m_xmax);
        yin_this = checkplace(m_points[i].m_y, r.m_ymin, r.m_ymax);
        if (xin_this == 0 && yin_this ==0
                             && m_points[i].m_t>=r.m_tmin && m_points[i].m_t<=r.m_tmax)
            return true;
        if (abs(xin_this + xin_last) < 2 && abs(yin_this + yin_last) < 2) {
            if (!searching_flag) {
                searching_flag = true;
                searchl = i-1;
            }
        } else {
            if(searching_flag) {
                intervals.emplace_back(std::make_pair(searchl, i - 1));
                searching_flag = false;
            }
        }
        if(i==h&&searching_flag){
            intervals.emplace_back(std::make_pair(searchl, i));
            searching_flag = false;
        }
        xin_last = xin_this;
        yin_last = yin_this;
    }
    int cursor1, cursor2 = m_tag + 1;
    for (auto &val:intervals) {
        cursor1 = val.first;
        cursor2 = locateTime(cursor2, m_points.size() - 1,
                             m_points[cursor1].m_t, 1);
        while (cursor1 < val.second) {
            while (m_points[cursor1 + 1].m_t > m_points[cursor2].m_t) {
                if(m_points[cursor2].m_t>r.m_tmax) break;
                xin_this = checkplace(m_points[cursor2].m_x, r.m_xmin,
                                      r.m_xmax);
                yin_this = checkplace(m_points[cursor2].m_y, r.m_ymin,
                                      r.m_ymax);
                if (xin_this == 0 && yin_this ==0
                                     && m_points[cursor2].m_t>=r.m_tmin)
                    return true;
                ++cursor2;
            }
            ++cursor1;
        }
    }
    return false;
     */
}



bool TBlock::passBy(MBR &r) {
    int l, h, ll, hh, cursor1, cursor2;
    l = locateTime(0, m_tag, r.m_tmin, -1);
    h = locateTime(l, m_tag, r.m_tmax, 1);
    cursor1 = l;
    if(m_tag+1!=m_points.size()) {
        ll = locateTime(m_tag + 1, m_points.size(), m_points[l].m_t, 1);
        hh = locateTime(ll, m_points.size(), m_points[h].m_t, -1);
        cursor2 = ll;
        int xin_last, yin_last, xin_this, yin_this;
        xin_last = checkplace(m_points[l].m_x, r.m_xmin, r.m_xmax);
        yin_last = checkplace(m_points[l].m_y, r.m_ymin, r.m_ymax);
        if (xin_last == 0 && yin_last == 0
            && m_points[l].m_t >= r.m_tmin && m_points[l].m_t <= r.m_tmax)
            return true;
        while (cursor2 <= hh) {
            cursor1++;
            xin_this = checkplace(m_points[cursor1].m_x, r.m_xmin, r.m_xmax);
            yin_this = checkplace(m_points[cursor1].m_y, r.m_ymin, r.m_ymax);
            if (xin_this == 0 && yin_this == 0 &&
                (cursor1 < h || (m_points[cursor1].m_t >= r.m_tmin &&
                                 m_points[cursor1].m_t <= r.m_tmax)))
                return true;
            if (m_points[cursor1].m_t < m_points[cursor2].m_t) {
                if(r.intersects(m_points[cursor1 - 1], m_points[cursor1]))
                    return true;
            }
            if (abs(xin_this + xin_last) >= 2 ||
                abs(yin_this + yin_last) >= 2) {
                while (cursor2 <= hh &&m_points[cursor1].m_t > m_points[cursor2].m_t) {
                    cursor2++;
                }
            } else {
                if (r.intersects(m_points[cursor1 - 1], m_points[cursor2]))
                    return true;
                while (cursor2 <= hh && m_points[cursor2 + 1].m_t < m_points[cursor1].m_t) {
                    if (r.intersects(m_points[cursor2],
                                     m_points[cursor2 + 1])) {
                        return true;
                    }
                    cursor2++;
                }
                if (cursor2<=hh && r.intersects(m_points[cursor2], m_points[cursor1]))
                    return true;
                cursor2++;
            }
            yin_last = yin_this;
            xin_last = xin_this;
        }
    }
    for (; cursor1 < h; cursor1++) {
        if (r.intersects(m_points[cursor1], m_points[cursor1+1]))
            return true;
    }
    return false;
}