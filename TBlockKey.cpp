//
// Created by Chuang on 2021/10/5.
//
#include "TBlockKey.hpp"
#include "Trajectory.hpp"
#include "vector"

using std::vector;


/*is p in block1(a,b)*/
inline bool inblock1(Point &a, Point &b, Point &p) {
    return ((p.m_x <= a.m_x && p.m_x >= b.m_x) ||
            (p.m_x >= a.m_x && p.m_x <= b.m_x))
           && ((p.m_y <= a.m_y && p.m_y >= b.m_y) ||
               (p.m_y >= a.m_y && p.m_y <= b.m_y));
}

inline bool inblock2(Point &a, Point &b, Point &p) {
    Point ra(a.m_x + a.m_y, a.m_x - a.m_y, a.m_t),
            rb(b.m_x + b.m_y, b.m_x - b.m_y, b.m_t),
            rp(p.m_x + p.m_y, p.m_x - p.m_y, p.m_t);
    return inblock1(ra, rb, rp);
}


double areaBlock(Point &a, Point &b, BType type) {
    if (type == T_block1) {
        return (fabs(b.m_x - a.m_x)) * (fabs(b.m_y - a.m_y));
    } else if (type == T_block2) {
        double du = fabs((b.m_x + b.m_y - (a.m_x + a.m_y))),
                dv = fabs((b.m_x - b.m_y - (a.m_x - a.m_y)));// /sqrt(2)
        return (du) * (dv) / 2;
    }
    return 1e300;
}

BType minType(BSize size) {
    BType res = T_box1;
    for (auto &t:{T_box2, T_block1, T_block2}) {
        if (size.size[t] < size.size[res])
            res = t;
    }
    return res;
}

double minSize(BSize size) {
    BType res = T_box1;
    for (auto &t:{T_box2, T_block1, T_block2}) {
        if (size.size[t] < size.size[res])
            res = t;
    }
    return size.size[res];
}

BSize blockSize(Trajectory &tj, int a, int b, BEnable ena) {
    if (a > b) {
        int tmp;
        tmp = a;
        a = b;
        b = tmp;
    }
    BSize size = {1e300, 1e300, 1e300, 1e300};

    //box1
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
    if (ena.enable[T_box1]) {
        xmin = xmax = tj.m_points[a].m_x;
        ymin = ymax = tj.m_points[a].m_y;
        for (int i = a + 1; i <= b; i++) {
            if (xmin > tj.m_points[i].m_x)
                xmin = tj.m_points[i].m_x;
            if (xmax < tj.m_points[i].m_x)
                xmax = tj.m_points[i].m_x;
            if (ymin > tj.m_points[i].m_y)
                ymin = tj.m_points[i].m_y;
            if (ymax < tj.m_points[i].m_y)
                ymax = tj.m_points[i].m_y;
        }
        size.size[T_box1] = (xmax - xmin) * (ymax - ymin);
    }


    // box2
    double umin = 0, umax = 0, vmin = 0, vmax = 0;
    if (ena.enable[T_box2]) {
        umin = umax = tj.m_points[a].m_x + tj.m_points[a].m_y;
        vmin = vmax = tj.m_points[a].m_x - tj.m_points[a].m_y;
        for (int i = a + 1; i <= b; i++) {
            double u = tj.m_points[i].m_x + tj.m_points[i].m_y;
            double v = tj.m_points[i].m_x - tj.m_points[i].m_y;
            if (umin > u)
                umin = u;
            if (umax < u)
                umax = u;
            if (vmin > v)
                vmin = v;
            if (vmax < v)
                vmax = v;
        }
        size.size[T_box2] = (umax - umin) * (vmax - vmin) / 2;
    }


    //block1
    if (ena.enable[T_block1]) {
        for (int i = a; i <= b; i++) {
            if (!inblock1(tj.m_points[a], tj.m_points[b], tj.m_points[i]))
                break;
            if (i == b) {
                size.size[T_block1] = areaBlock(tj.m_points[a], tj.m_points[b],
                                                T_block1);
            }
        }
    }

    //block2
    if (ena.enable[T_block2]) {
        for (int i = a; i <= b; i++) {
            if (!inblock2(tj.m_points[a], tj.m_points[b], tj.m_points[i]))
                break;
            if (i == b) {
                size.size[T_block2] = areaBlock(tj.m_points[a], tj.m_points[b],
                                                T_block2);
            }
        }
    }

    return size;
}

vector<TBlockRoute> OPTcost(Trajectory &tj, BEnable ena, int numbox) {
    /*
     * DP table
     * |the first point| using one box| using two box | ...
     * |the second point| using one box| using two box | ...
     */
    vector<vector<TBlockRoute>> dmat;
    dmat.resize(tj.m_points.size());
    int i = 1, j, k;
    for (auto &vec: dmat) {
        vec.resize(i); /*n+1 points can at most be expressed by n boxes*/
        for (auto &d:vec) {
            d.cost = 1e300;
        }
        i++;
    }
    BSize s = {1e300, 1e300, 1e300, 1e300};;
    for (i = 1; i < tj.m_points.size(); i++) {// last id of points
        for (k = 1; k <= std::min(i, numbox); k++) { // num of boxes
            if (k == 1) {
                j = 0;
                s = blockSize(tj, i, j, ena);
                dmat[i][k].m_route.emplace_back(
                        TBlockRouteEntry(j, i, minType(s), s));
                dmat[i][k].cost = minSize(s);
            } else {
                for (j = 0; j < i; j++) { //last chosen point
                    if (j >= k - 1) {
                        s = blockSize(tj, i, j, ena);
                        double value = dmat[j][k - 1].cost + minSize(s);
                        if (dmat[i][k].cost > value) {
                            dmat[i][k].cost = value;
                            dmat[i][k].m_route = dmat[j][k - 1].m_route;
                            dmat[i][k].m_route.emplace_back(
                                    TBlockRouteEntry(j, i, minType(s), s));
                        }
                    }
                }
            }
        }
    }
    return dmat.back();
}

TBlockRoute GreedyPath(Trajectory &tj, BEnable ena) {
    int startloc = 0;
    double x, y;
    double u, v;
    TBlockRoute res;

    bool xyena = ena.enable[T_block1], uvena = ena.enable[T_block2];
    int directionx = 0, directiony = 0; //-1 is decrease, 0 is unknown, 1 is increase
    int directionu = 0, directionv = 0; //-1 is decrease, 0 is unknown, 1 is increase
    for (int i = 0; i < tj.m_points.size(); i++) {
        if (i == 0) {
            x = tj.m_points[i].m_x;
            y = tj.m_points[i].m_y;
            u = x + y;
            v = x - y;
            directionx = 0;
            directiony = 0;
            continue;
        }
        double newx = tj.m_points[i].m_x, newy = tj.m_points[i].m_y,
                newu = newx + newy, newv = newx - newy;
        if (xyena) {
            if (
                    (directionx == 1 && newx < x)
                    || (directionx == -1 && newx > x)
                    || (directiony == 1 && newy < y)
                    || (directiony == -1 && newy > y)
                    ) {
                xyena = false;
                directionx = 0;
                directiony = 0;
            }
            if (directionx == 0) {
                if (newx > x) directionx = 1;
                else if (newx < x) directionx = -1;
            }
            if (directiony == 0) {
                if (newy > y) directiony = 1;
                else if (newy < y) directiony = -1;
            }
            x = newx;
            y = newy;
        }
        if (uvena) {
            if (
                    (directionu == 1 && newu < u)
                    || (directionu == -1 && newu > u)
                    || (directionv == 1 && newv < v)
                    || (directionv == -1 && newv > v)
                    ) {
                uvena = false;
                directionu = 0;
                directionv = 0;
            }
            if (directionu == 0) {
                if (newu > u) directionu = 1;
                else if (newu < u) directionu = -1;
            }
            if (directionv == 0) {
                if (newv > v) directionv = 1;
                else if (newv < v) directionv = -1;
            }
            u = newu;
            v = newv;
        }
        if (!xyena && !uvena) {
            x = tj.m_points[i - 1].m_x;
            y = tj.m_points[i - 1].m_y;
            u = x + y;
            v = x - y;
            BSize s = blockSize(tj, startloc, i - 1, ena);
            res.m_route.emplace_back(
                    TBlockRouteEntry(startloc, i - 1, minType(s), s));
            res.cost += minSize(s);
            startloc = i - 1;
        }
        if (i == tj.m_points.size() - 1) {
            BSize s = blockSize(tj, startloc, i, ena);
            res.m_route.emplace_back(
                    TBlockRouteEntry(startloc, i, minType(s), s));
            res.cost += minSize(s);
        }
    }
    return res;
}

#include <queue>

std::tuple<int, BSize, BSize>
picksplitpoint(Trajectory &tj, int s, int e, BEnable ena) {
    double size = 1e300;
    int mini;
    BSize bests1, bests2;
    for (int i = s + 1; i < e; i++) {
        BSize s1 = blockSize(tj, s, i, ena);
        BSize s2 = blockSize(tj, i, e, ena);
        double d = minSize(s1) + minSize(s2);
        if (d < size) {
            mini = i;
            bests1 = s1;
            bests2 = s2;
        }
    }
    return std::make_tuple(mini, bests1, bests2);
}

TBlockRoute GreedyPathElite(Trajectory &tj, BEnable ena, int numseg) {
    auto greed = GreedyPath(tj, ena);
    if (numseg <= greed.m_route.size())
        return greed;
    std::priority_queue<TBlockRouteEntry, vector<TBlockRouteEntry>, TBlockRouteEntry::TBRE_cmp_size> pq;
    for (auto &s:greed.m_route) {
        pq.push(s);
    }
    auto top = pq.top();
    pq.pop();
    if (top.m_pe.m_plast - top.m_ps.m_plast > 1) {
        auto s = picksplitpoint(tj, top.m_ps.m_plast, top.m_pe.m_plast, ena);
        pq.push(TBlockRouteEntry(top.m_ps, std::get<0>(s),
                                 minType(std::get<1>(s)), std::get<1>(s)));
        pq.push(TBlockRouteEntry(std::get<0>(s), top.m_pe,
                                 minType(std::get<2>(s)), std::get<2>(s)));
    } else {
        IntRange mid = top.m_ps;
        mid.m_ratio =
                (top.m_ps.m_ratio + top.m_pe.m_ratio == 0 ? 1 : top.m_pe.m_ratio)/2;
        BSize msize = top.m_size;
        msize.size[0] /= 4;
        msize.size[1] /= 4;
        msize.size[2] /= 4;
        msize.size[3] /= 4;
        pq.push(TBlockRouteEntry(top.m_ps, mid, top.m_type, msize));
        pq.push(TBlockRouteEntry(mid, top.m_pe, top.m_type, msize));
    }
    std::priority_queue<TBlockRouteEntry, vector<TBlockRouteEntry>, TBlockRouteEntry::TBRE_cmp_ps> sortpq;
    while(!pq.empty())
    {
        sortpq.push(pq.top());
        pq.pop();
    }
    TBlockRoute res;
    while(!sortpq.empty())
    {
        res.m_route.emplace_back(sortpq.top());
        res.cost += sortpq.top().m_size.size[sortpq.top().m_type];
        sortpq.pop();
    }
    return res;
}


//TBlockKey OPTBlock(Trajectory &tj, int nbox, BEnable ena) {
//    /*
//     * DP table
//     * |the first point| using one box| using two box | ...
//     * |the second point| using one box| using two box | ...
//     */
//    vector<vector<double>> dmat; // cost
//    vector<vector<BType>> dtype; // the block type of previous block
//    vector<vector<int>> dprev; // the index of previous block
//    dmat.resize(tj.m_points.size());
//    dtype.resize(tj.m_points.size());
//    dprev.resize(tj.m_points.size());
//    int i = 0, j, k;
//    for (auto &vec: dmat) {
//        vec.resize(std::min(i, nbox));
//        for (auto &d:vec) {
//            d = 1e300;
//        }
//        i++;
//    }
//    BSize s = {1e300, 1e300, 1e300, 1e300};;
//    for (i = 1; i < tj.m_points.size(); i++) {// num of point
//        for (k = 1; k <= std::min(i, nbox); k++) { // num of boxes
//            if (k == 1) {
//                j = 0;
//                s = blockSize(tj, i, j, ena);
//                dmat[i][k] = minSize(s);
//            }
//            for (j = 0; j < i; j++) { //last point
//                s = blockSize(tj, i, j, ena);
//                BType type = minType(s);
//                double value = dmat[i][k - 1] + minSize(s);
//                if (dmat[i][k] > value) {
//                    dmat[i][k] = value;
//                    dprev[i][k] = j;
//                    dtype[i][k] = type;
//                }
//            }
//        }
//    }
//    TBlockKey res;
//    vector<int> rev;
//    int cur = tj.m_points.size();
//    int curbox = nbox;
//    while (cur != 0) {
//        rev.emplace_back(cur);
//        cur = dprev[cur][curbox - 1];
//    }
//    rev.emplace_back(0);
//    res.m_size = rev.size() - 1;
//    for (int i = rev.size(); i >= 0; i--) {
//        res.m_type.emplace_back(dtype[rev[i]][i]);
//        res.m_pts.emplace_back(tj.m_points[rev[i]]);
//    }
//    return res;
//}

