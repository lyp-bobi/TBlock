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

bool greedy_path = false;

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

    TBlockRoute c;

    if (!greedy_path) {
        c = OPTcost(t, ena, numseg)[numseg];
    } else {
        c = GreedyBox(t, ena, numseg);
    }

    BOUND_BOX_2D *see = NULL;
    for (int i = 0;i<numseg;i++)
    {
        see = (BOUND_BOX_2D*)&(res->BL_data[i]);
        res->BL_data[i].B_type = BT_box1;
		see->xmin = see->ymin = FLT_MAX;
		see->xmax = see->ymax = -FLT_MAX;
		for ( int j = c.m_route[i].m_ps.m_plast; j <= c.m_route[i].m_pe.m_plast; j++)
		{
			if(j == c.m_route[i].m_ps.m_plast){
				see->xmin = std::min(see->xmin, (float)t[c.m_route[i].m_ps].m_x);
				see->xmax = std::max(see->xmax, (float)t[c.m_route[i].m_ps].m_x);
				see->ymin = std::min(see->ymin, (float)t[c.m_route[i].m_ps].m_y);
				see->ymax = std::max(see->ymax, (float)t[c.m_route[i].m_ps].m_y);
			}
			else if(j == c.m_route[i].m_pe.m_plast)
			{
				see->xmin = std::min(see->xmin, (float)t[c.m_route[i].m_pe].m_x);
				see->xmax = std::max(see->xmax, (float)t[c.m_route[i].m_pe].m_x);
				see->ymin = std::min(see->ymin, (float)t[c.m_route[i].m_pe].m_y);
				see->ymax = std::max(see->ymax, (float)t[c.m_route[i].m_pe].m_y);
			}
			else
			{
				see->xmin = std::min(see->xmin, (float)t[j].m_x);
				see->xmax = std::max(see->xmax, (float)t[j].m_x);
				see->ymin = std::min(see->ymin, (float)t[j].m_y);
				see->ymax = std::max(see->ymax, (float)t[j].m_y);
			}
		}
    }
    return res;
}
BOUNDLIST* BOUNDPRODUCER::produce_tblock_list(int numseg)
{
    double *dlist = (double *) (m_ptarray->serialized_pointlist);
	int dim = FLAGS_NDIMS(m_ptarray->flags);
    Trajectory t;
	double xmin, xmax, ymin, ymax;
	xmin = ymin = FLT_MAX;
	xmax = ymax = -FLT_MAX;
	for (int i = 0; i < m_ptarray->npoints; i++) {
		t.m_points.emplace_back(
				Point(dlist[dim * i], dlist[dim * i + 1], i));
		if(dlist[dim * i] < xmin)
			xmin = dlist[dim * i];
		if(dlist[dim * i + 1] < ymin)
			ymin = dlist[dim * i + 1];
		if(dlist[dim * i] > xmax)
			xmax = dlist[dim * i];
		if(dlist[dim * i + 1] > ymax)
			ymax = dlist[dim * i + 1];
	}
    if(t.m_points.size() <= numseg)
    {
        t.resample(numseg+1);
    }
    BEnable ena = {false, false, true, true};
    TBlockRoute c;
    if (!greedy_path) {
        c = OPTcostMin(t, ena);
        if (c.cost > 1e299) {
            c = GreedyPath(t, ena);
        }
    } else {
//        c = GreedyPath(t, ena);
        c = GreedyPathMod(t, ena, numseg);
    }
    numseg = c.m_route.size();
	BOUNDLIST *res = static_cast<BOUNDLIST *>(malloc(
            sizeof(BOUNDLIST) + sizeof(BOUND) * numseg));
    res->BL_numbox = numseg;
    res->BL_type = BLT_blocklist;
	res->xmin = xmin;
	res->xmax = xmax;
	res->ymin = ymin;
	res->ymax = ymax;
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
        else
        {
            throw std::runtime_error("should be block");
        }
    }
    return res;
}
