//
// Created by Chuang on 2022/7/3.
//

#include <TBlock_c.h>
#include <TBlockKey.h>
#include <cfloat>

using std::vector;

POINTARRAY *tbox_opt(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {true, false, false, false};
    TBlockKey c = OPTBlock(t, numseg, ena);
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + c.m_size * sizeof(double) * 3);
    res->npoints = c.m_size;
    dlist = (double *) (res->serialized_pointlist);
    for (int i = 0; i < c.m_size; i++) {
        dlist[3 * i] = c.m_pts[i].m_x;
        dlist[3 * i + 1] = c.m_pts[i].m_y;
        dlist[3 * i + 2] = c.m_pts[i].m_t;
    }
    return res;
}

POINTARRAY *tblock_opt(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {false, false, true, true};
    auto c = OPTcost(t, ena, numseg)[numseg];
    if (c.cost > 1e299) {
        c = GreedyPath(t, ena);
    }
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + c.m_route.size() * sizeof(double) * 3);
    res->npoints = +c.m_route.size();
    dlist = (double *) (res->serialized_pointlist);
    int i;
    for (i = 0; i < c.m_route.size() - 1; i++) {
        dlist[3 * i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3 * i + 1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3 * i + 2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3 * i] = t.m_points[c.m_route[i].m_pe.m_plast].m_x;
    dlist[3 * i + 1] = t.m_points[c.m_route[i].m_pe.m_plast].m_y;
    dlist[3 * i + 2] = t.m_points[c.m_route[i].m_pe.m_plast].m_t;
    return res;
}

POINTARRAY *tblock_greedy(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {false, false, true, true};
    auto c = GreedyPathElite(t, ena, numseg);
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + (c.m_route.size() + 1) * sizeof(double) * 3);
    res->npoints = c.m_route.size() + 1;
    dlist = (double *) (res->serialized_pointlist);
    int i = 0;
    for (; i < c.m_route.size(); i++) {
        dlist[3 * i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3 * i + 1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3 * i + 2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3 * i] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_x;
    dlist[3 * i + 1] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_y;
    dlist[3 * i + 2] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_t;
    return res;
}

//varlena* tbox_opt_to_binary(POINTARRAY *traj, int numseg)
//{
//    double *dlist = (double *) (traj->serialized_pointlist);
//    Trajectory t;
//    for (int i = 0; i < traj->npoints; i++) {
//        t.m_points.emplace_back(
//                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
//    }
//    BEnable ena = {true, false, false, false};
//    vector<TBlockRoute> route = OPTcost(t, ena, numseg);
//    TBOX_SERL* res = (TBOX_SERL*)malloc(TBOX_NBOX_TO_VARSIZE(numseg));
//    SET_VARSIZE(res, TBOX_NBOX_TO_VARSIZE(numseg));
//    int cur = 0;
//    for(auto &b:route[numseg].m_route)
//    {
//        res->coords[cur++]=t[b.m_ps].m_x;
//        res->coords[cur++]=t[b.m_ps].m_y;
//        res->coords[cur++]=t[b.m_ps].m_t;
//        res->coords[cur++]=t[b.m_pe].m_x;
//        res->coords[cur++]=t[b.m_pe].m_y;
//        res->coords[cur++]=t[b.m_pe].m_t;
//    }
//    return (varlena*) res;
//}
//
//varlena* tblock_opt_to_binary(POINTARRAY *traj, int numseg)
//{
//    double *dlist = (double *) (traj->serialized_pointlist);
//    Trajectory t;
//    for (int i = 0; i < traj->npoints; i++) {
//        t.m_points.emplace_back(
//                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
//    }
//    BEnable ena = {false, false, true, true};
//    TBlockRoute route = GreedyPathElite(t,ena,numseg);
//    numseg = route.m_route.size();
//    TBLOCK_SERL* res = (TBLOCK_SERL*)malloc(TBLOCK_NBOX_TO_VARSIZE(numseg));
//    SET_VARSIZE(res, TBOX_NBOX_TO_VARSIZE(numseg));
//    int cur1 = 0,cur2 = 0;
//    int len_type = numseg;
//    int len_coord = (numseg+1) * (sizeof(float)*3);
//    char* types = (char*)malloc(len_type);
//    char* coords = (char*)malloc(len_coord);
//    for(auto &b:route.m_route)
//    {
//        types[cur1++] = b.m_type - T_block1;
//        coords[cur2++] = t[b.m_ps].m_x;
//        coords[cur2++] = t[b.m_ps].m_y;
//        coords[cur2++] = t[b.m_ps].m_t;
//    }
//    coords[cur2++] = t[route.m_route.back().m_pe].m_x;
//    coords[cur2++] = t[route.m_route.back().m_pe].m_y;
//    coords[cur2++] = t[route.m_route.back().m_pe].m_t;
//    memcpy(res->data,types,len_type);
//    memcpy(res->data+len_type, coords, len_coord);
//    return (varlena*) res;
//}
//
//TBlock_c binary_to_tbox(varlena* data)
//{
//    TBOX_SERL *bserl = (TBOX_SERL*) data;
//    int numbox = TBOX_VARSIZE_TO_NBOX(VARSIZE(bserl));
//    TBlock_c ret;
//    ret.m_size = numbox;
//    ret.m_type = static_cast<BType *>(malloc(sizeof(BType) * numbox));
//    ret.m_point
//    for(int i=0;i<numbox;i++)
//    {
//
//    }
//}
//TBlock_c binary_to_tblock(varlena* data);