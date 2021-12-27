//
// Created by Chuang on 2021/6/2.
//

#include "Trajectory.h"
double timer_traj = 0;

std::vector<std::string> split(const std::string &strtem, char a) {
    std::vector<std::string> strvec;

    std::string::size_type pos1, pos2;
    pos2 = strtem.find(a);
    pos1 = 0;
    while (std::string::npos != pos2) {
        strvec.emplace_back(strtem.substr(pos1, pos2 - pos1));

        pos1 = pos2 + 1;
        pos2 = strtem.find(a, pos1);
    }
    strvec.emplace_back(strtem.substr(pos1));
    return strvec;
}

std::string Trajectory::toString() const {
    std::string s = "";
    for (const auto &p:m_points) {
        s += std::to_string(p.m_x) + "," + std::to_string(p.m_y) +
             "," + std::to_string(p.m_t) + " ";
    }
    if (s.length() > 0) s[s.length() - 1] = '\0';
    return s;
}

void Trajectory::loadFromString(std::string str) {
    m_points.clear();
    std::vector<std::string> points = split(str, ' ');
    for (const auto &p: points) {
        std::vector<std::string> xyt = split(p, ',');
        m_points.emplace_back(
                Point(std::stod(xyt[0]), std::stod(xyt[1]), std::stod(xyt[2])));
    }
    if (m_points.size() > 1 && m_points.front().m_t >= m_points.back().m_t) {
        m_points.clear();
        throw std::logic_error("");
    }
}

MBR Trajectory::getMBR() {
    MBR res(1e30, -1e30, 1e30, -1e30, 1e30, -1e30);
    for(int i=0;i<m_points.size();i++){
        res.combinePoint(m_points[i]);
    }
    return res;
}

inline int Trajectory::locateTime(int l, int h, double time, int dir) {
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

bool Trajectory::stepIn(MBR &r) {
    if (r.m_tmax < m_points.front().m_t || r.m_tmin > m_points.back().m_t) {
        return false;
    }
    int l = locateTime(0, m_points.size() - 1, r.m_tmin, 1),
            h = locateTime(std::max(0,l-1),m_points.size() -1,r.m_tmax,-1);
    for (int i = l; i <= h; i++) {
        if (m_points[i].m_x >= r.m_xmin && m_points[i].m_x <= r.m_xmax
            && m_points[i].m_y >= r.m_ymin && m_points[i].m_y <= r.m_ymax
            )
            return true;
    }
    return false;
}

bool Trajectory::passBy(MBR &r) {
    if (r.m_tmax < m_points.front().m_t || r.m_tmin > m_points.back().m_t) {
        return false;
    }
    int l = locateTime(0, m_points.size() - 1, r.m_tmin, -1),
            h = locateTime(l,m_points.size() -1,r.m_tmax,1);
    for (int i = l; i <= h; i++) {
        if (m_points[i].m_x >= r.m_xmin && m_points[i].m_x <= r.m_xmax
            && m_points[i].m_y >= r.m_ymin && m_points[i].m_y <= r.m_ymax
                )
            return true;
    }
    for (int i = l+1; i <= h; i++) {
        if(r.intersects(m_points[i-1], m_points[i]))
            return true;
    }
    return false;
}