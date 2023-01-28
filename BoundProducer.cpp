//
// Created by Chuang on 2022/9/16.
//
#include <vector>
#include <string>
#include <sstream>
#include <TBlockKey.hpp>
#include <cfloat>

#include <BoundProducer.hpp>
using std::vector;
using std::string;

/**
 * @brief only support 2d now
 * @param numseg 
 * @return 
 */
BOUNDLIST* BOUNDPRODUCER::produce_tbox_list(int numseg)
{
    double *dlist = (double *) (m_ptarray->serialized_pointlist);
    Trajectory t;
	int dim = FLAGS_NDIMS(m_ptarray->flags);
    BOUNDLIST *res = static_cast<BOUNDLIST *>(malloc(
            sizeof(BOUNDLIST) + sizeof(BOUND) * numseg));
    res->xmin = res->ymin = FLT_MAX;
    res->xmax = res->ymax = -FLT_MAX;
    res->BL_numbox = numseg;
    res->BL_type = BLT_boxlist;

    for (int i = 0; i < m_ptarray->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[dim * i], dlist[dim * i + 1], i));
        if(dlist[dim * i] < res->xmin)
            res->xmin = dlist[dim * i];
        if(dlist[dim * i + 1] < res->ymin)
            res->ymin = dlist[dim * i + 1];
        if(dlist[dim * i] > res->xmax)
            res->xmax = dlist[dim * i];
        if(dlist[dim * i + 1] > res->ymax)
            res->ymax = dlist[dim * i + 1];
    }
    if(t.m_points.size() <= numseg)
    {
        t.resample(numseg+1);
    }
    BEnable ena = {true, false, false, false};
    TBlockRoute c = OPTcost(t, ena, numseg)[numseg];

    BOUND_BOX_2D *see = NULL;
    for (int i = 0;i<numseg;i++)
    {
        see = (BOUND_BOX_2D*)&(res->BL_data[i]);
        res->BL_data->B_type = BT_box1;
        see->xmin = std::min(t[c.m_route[i].m_ps].m_x, t[c.m_route[i].m_pe].m_x);
        see->xmax = std::max(t[c.m_route[i].m_ps].m_x, t[c.m_route[i].m_pe].m_x);
        see->ymin = std::min(t[c.m_route[i].m_ps].m_y, t[c.m_route[i].m_pe].m_y);
        see->ymax = std::max(t[c.m_route[i].m_ps].m_y, t[c.m_route[i].m_pe].m_y);
    }
    return res;
}
BOUNDLIST* BOUNDPRODUCER::produce_tblock_list(int numseg)
{
    double *dlist = (double *) (m_ptarray->serialized_pointlist);
	int dim = FLAGS_NDIMS(m_ptarray->flags);
    Trajectory t;
    BOUNDLIST *res = static_cast<BOUNDLIST *>(malloc(
            sizeof(BOUNDLIST) + sizeof(BOUND) * numseg));
    res->BL_numbox = numseg;
    res->BL_type = BLT_blocklist;
    for (int i = 0; i < m_ptarray->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[dim * i], dlist[dim * i + 1], i));
        if(dlist[dim * i] < res->xmin)
            res->xmin = dlist[dim * i];
        if(dlist[dim * i + 1] < res->ymin)
            res->ymin = dlist[dim * i + 1];
        if(dlist[dim * i] > res->xmax)
            res->xmax = dlist[dim * i];
        if(dlist[dim * i + 1] > res->ymax)
            res->ymax = dlist[dim * i + 1];
    }
    if(t.m_points.size() <= numseg)
    {
        t.resample(numseg+1);
    }
    BEnable ena = {false, false, true, true};
    TBlockRoute c = OPTcost(t, ena, numseg)[numseg];
    if (c.cost > 1e299) {
        c = GreedyPath(t, ena);
    }
    numseg = c.m_route.size() - 1;
    BOUND_BLOCK1_2D *see1 = NULL;
    BOUND_BLOCK2_2D *see2 = NULL;
    for (int i = 0;i<numseg;i++)
    {
        if(c.m_route[i].m_type == T_block1)
        {
            res->BL_data[i].B_type = BT_block1;
            see1 = (BOUND_BLOCK1_2D*)&(res->BL_data[i]);
            see1->xs = t[c.m_route[i].m_ps].m_x;
            see1->xe = t[c.m_route[i].m_pe].m_x;
            see1->ys = t[c.m_route[i].m_ps].m_y;
            see1->ye = t[c.m_route[i].m_pe].m_y;
        }
        else if(c.m_route[i].m_type == T_block2)
        {
            res->BL_data[i].B_type = BT_block2;
            see2 = (BOUND_BLOCK2_2D*)&(res->BL_data[i]);
            see2->xs = t[c.m_route[i].m_ps].m_x;
            see2->xe = t[c.m_route[i].m_pe].m_x;
            see2->ys = t[c.m_route[i].m_ps].m_y;
            see2->ye = t[c.m_route[i].m_pe].m_y;
        }
    }
    return res;
}
