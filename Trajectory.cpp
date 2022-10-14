//
// Created by Chuang on 2021/6/2.
//

#include "Trajectory.hpp"
#include <queue>
#include <map>
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

Point Trajectory::operator[](IntRange r)
{
    double x = m_points[r.m_plast].m_x * (1-r.m_ratio) + m_points[r.m_plast+1].m_x * r.m_ratio;
    double y = m_points[r.m_plast].m_y * (1-r.m_ratio) + m_points[r.m_plast+1].m_y * r.m_ratio;
    double t = m_points[r.m_plast].m_t * (1-r.m_ratio) + m_points[r.m_plast+1].m_t * r.m_ratio;
    return Point(x,y,t);
}

POINTARRAY * Trajectory::asptarray() {
    POINTARRAY *res = static_cast<POINTARRAY *>(malloc(
            sizeof(POINTARRAY)));
    res->npoints = m_points.size();
    res->maxpoints = res->npoints;
    res->serialized_pointlist = (uint8_t*) malloc(2 * sizeof(double) * m_points.size());
    double * data = (double*)(res->serialized_pointlist);
    int cur = 0;
    for(auto &p:m_points)
    {
        data[cur++]=p.m_x;
        data[cur++] = p.m_y;
    }
    return res;
}

void Trajectory::resample(int numseg) {
    if(numseg > m_points.size())
    {
        int point_add = numseg - m_points.size();
        std::priority_queue<std::tuple<double, int, int>, std::vector<std::tuple<double, int, int>>, std::greater<std::tuple<double, int, int>>> queue;
        for(int i=0;i<m_points.size() -1 ;i++)
        {
            queue.push(std::make_tuple(m_points[i+1].getDistance(m_points[i]),1,i));
        }
        std::map<int,int> inserted;
        for(int j =0;j<point_add;j++)
        {
            auto t = queue.top();
            queue.pop();
            queue.push(std::make_tuple(
                    std::get<0>(t)*std::get<1>(t) / (std::get<1>(t)+1)
                            ,std::get<1>(t)+1
                                    , std::get<2>(t)));
            inserted[std::get<2>(t)] = std::get<1>(t)+1;
        }
        std::vector<Point> res;
        for(int i=0;i<m_points.size();i++)
        {
            if(inserted.find(i) == inserted.end())
            {
                res.emplace_back(m_points[i]);
            } else{
                for(int j =0; j < inserted[i];j++)
                {
                    double ratio = 1.0*j/inserted[i];
                    res.emplace_back(makemid(m_points[i], m_points[i+1], m_points[i].m_t * (1 - ratio)+m_points[i+1].m_t*ratio));
                }
            }
        }
        m_points.swap(res);
    }
}