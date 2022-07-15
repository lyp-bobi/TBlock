//
// Created by Chuang on 2022/7/3.
//

#include <TBlock_c.h>
#include <TBlockKey.h>

POINTARRAY* tbox_opt(POINTARRAY *traj, int numseg)
{
    double *dlist = (double*)(traj->serialized_pointlist);
    Trajectory t;
    for(int i = 0; i<traj->npoints; i++)
    {
        t.m_points.emplace_back(Point(dlist[3*i], dlist[3*i+1], dlist[3*i+2]));
    }
    BEnable ena = {true,false, false, false};
    TBlockKey c = OPTBlock(t, numseg, ena);
    POINTARRAY* res = (POINTARRAY*) malloc(sizeof(POINTARRAY)+c.m_size*sizeof(double)*3);
    res->npoints = c.m_size;
    dlist = (double*)(res->serialized_pointlist);
    for(int i=0;i<c.m_size;i++)
    {
        dlist[3*i] = c.m_pts[i].m_x;
        dlist[3*i+1] = c.m_pts[i].m_y;
        dlist[3*i+2] = c.m_pts[i].m_t;
    }
    return res;
}
POINTARRAY* tblock_opt(POINTARRAY *traj, int numseg)
{
    double *dlist = (double*)(traj->serialized_pointlist);
    Trajectory t;
    for(int i = 0; i<traj->npoints; i++)
    {
        t.m_points.emplace_back(Point(dlist[3*i], dlist[3*i+1], dlist[3*i+2]));
    }
    BEnable ena = {false,false, true, true};
    auto c = OPTcost(t, ena, numseg)[numseg];
    if(c.cost > 1e299)
    {
        c = GreedyPath(t,ena);
    }
    POINTARRAY* res = (POINTARRAY*) malloc(sizeof(POINTARRAY)+c.m_route.size()*sizeof(double)*3);
    res->npoints = +c.m_route.size();
    dlist = (double*)(res->serialized_pointlist);
    int i;
    for(i=0;i<c.m_route.size()-1;i++)
    {
        dlist[3*i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3*i+1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3*i+2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3*i] = t.m_points[c.m_route[i].m_pe.m_plast].m_x;
    dlist[3*i+1] = t.m_points[c.m_route[i].m_pe.m_plast].m_y;
    dlist[3*i+2] = t.m_points[c.m_route[i].m_pe.m_plast].m_t;
    return res;
}
POINTARRAY* tblock_greedy(POINTARRAY *traj, int numseg)
{
    double *dlist = (double*)(traj->serialized_pointlist);
    Trajectory t;
    for(int i = 0; i<traj->npoints; i++)
    {
        t.m_points.emplace_back(Point(dlist[3*i], dlist[3*i+1], dlist[3*i+2]));
    }
    BEnable ena = {false,false, true, true};
    auto c = GreedyPathElite(t, ena, numseg);
    POINTARRAY* res = (POINTARRAY*) malloc(sizeof(POINTARRAY)+(c.m_route.size()+1)*sizeof(double)*3);
    res->npoints = c.m_route.size() + 1;
    dlist = (double*)(res->serialized_pointlist);
    int i = 0;
    for(;i<c.m_route.size();i++)
    {
        dlist[3*i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3*i+1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3*i+2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3*i] = t.m_points[c.m_route[i-1].m_pe.m_plast].m_x;
    dlist[3*i+1] = t.m_points[c.m_route[i-1].m_pe.m_plast].m_y;
    dlist[3*i+2] = t.m_points[c.m_route[i-1].m_pe.m_plast].m_t;
    return res;
}