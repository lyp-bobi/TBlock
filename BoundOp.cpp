//
// Created by Chuang on 2023/3/21.
//
#ifdef __cplusplus
extern "C"
{
#endif
#include <Bound.h>
#include <BoundOp.h>
#ifdef __cplusplus
}
#endif
#include <Trajectory.hpp>


double bound_b_p_mindist_square(BOUND* b, POINT2D p)
{
    if(b->B_type == BT_box1)
    {
        double ret = 0;
        BOUND_BOX_2D *b1 = (BOUND_BOX_2D*)b;
        double xmin = b1->xmin, xmax = b1->xmax,
                ymin = b1->ymin, ymax = b1->ymax;
        if(p.x > xmax)
        {
            ret += (p.x - xmax) * (p.x - xmax);
        }
        if(p.x < xmin)
        {
            ret += (p.x - xmin) * (p.x - xmin);
        }
        if(p.y > ymax)
        {
            ret += (p.y - ymax) * (p.x - ymax);
        }
        if(p.y < ymin)
        {
            ret += (p.y - ymin) * (p.y - ymin);
        }
        return ret;
    }
    if(b->B_type == BT_block1)
    {
        double ret = 0;
        BOUND_BLOCK1_2D *b1 = (BOUND_BLOCK1_2D*)b;
        double xmin = std::min(b1->xs,b1->xe), xmax = std::max(b1->xs,b1->xe),
                ymin = std::min(b1->ys,b1->ye), ymax = std::max(b1->ys,b1->ye);
        if(p.x > xmax)
        {
            ret += (p.x - xmax) * (p.x - xmax);
        }
        if(p.x < xmin)
        {
            ret += (p.x - xmin) * (p.x - xmin);
        }
        if(p.y > ymax)
        {
            ret += (p.y - ymax) * (p.x - ymax);
        }
        if(p.y < ymin)
        {
            ret += (p.y - ymin) * (p.y - ymin);
        }
        return ret;
    }
    else if(b->B_type == BT_block2)
    {
        double ret = 0;
        double pu = p.x + p.y, pv = p.x - p.y;
        BOUND_BLOCK2_2D *b1 = (BOUND_BLOCK2_2D*)b;
        double us = b1->xs + b1->ys, vs = b1->xs - b1->ys,
                ue = b1->xe + b1->ye, ve = b1->xe - b1->ye;
        double umin = std::min(us, ue), umax = std::max(us, ue),
                vmin = std::min(vs, ve), vmax = std::max(vs, ve);
        if(pu > umax)
        {
            ret += (pu - umax) * (pu - umax);
        }
        if(pu < umin)
        {
            ret += (pu - umin) * (pu - umin);
        }
        if(pv > vmax)
        {
            ret += (pu - vmax) * (pu - vmax);
        }
        if(pv < vmin)
        {
            ret += (pv - vmin) * (pv - vmin);
        }
        return ret/2;
    }
}

double dtw_lb(POSTGIS_POINTARRAY *ps, BOUNDLIST *bl)
{
    double res = 0;
    Trajectory tj(ps);
    for(int i = 0; i < tj.m_points.size(); i++)
    {
        POINT2D p;
        p.x = tj.m_points[i].m_x;
        p.y = tj.m_points[i].m_y;
        double mindist_square = 1e300;
        for(int j = 0; j < bl->BL_numbox ; j++)
        {
            double dist = bound_b_p_mindist_square(&(bl->BL_data[j]), p);
            if(dist < mindist_square)
                mindist_square = dist;
        }
        res += sqrt(mindist_square);
    }
    return res;
}