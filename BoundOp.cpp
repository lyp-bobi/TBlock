//
// Created by Chuang on 2023/3/21.
//
#ifdef __cplusplus
extern "C"
{
#endif
#include <Bound.h>
#include <BoundOp.h>
#include <float.h>
#ifdef __cplusplus
}
#endif
#include <Trajectory.hpp>


double bound_b_p_mindist_square(BOUND* b, POSTGIS_POINT2D p)
{
    if(b->B_type == BT_box1)
    {
        double ret = 0;
        BOUND_BOX_2D *b1 = (BOUND_BOX_2D*)b;
        double xmin = nextafterf(b1->xmin, -FLT_MAX), xmax = nextafterf(b1->xmax, FLT_MAX),
                ymin = nextafterf(b1->ymin, -FLT_MAX), ymax = nextafterf(b1->ymax, FLT_MAX);
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
            ret += (p.y - ymax) * (p.y - ymax);
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
        double xmin = nextafterf(std::min(b1->xs,b1->xe), -FLT_MAX), xmax = nextafterf(std::max(b1->xs,b1->xe), FLT_MAX),
                ymin = nextafterf(std::min(b1->ys,b1->ye), -FLT_MAX), ymax = nextafterf(std::max(b1->ys,b1->ye), FLT_MAX);
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
            ret += (p.y - ymax) * (p.y - ymax);
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
        double umin = nextafterf(std::min(us, ue), -FLT_MAX), umax = nextafterf(std::max(us, ue), FLT_MAX),
                vmin = nextafterf(std::min(vs, ve), -FLT_MAX), vmax = nextafterf(std::max(vs, ve), FLT_MAX);
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
            ret += (pv - vmax) * (pv - vmax);
        }
        if(pv < vmin)
        {
            ret += (pv - vmin) * (pv - vmin);
        }
        return ret/2;
    } else
    {
        return 0;
    }
}

double bound_bl_bl_mindist_fast_square(BOUNDLIST* bl1, BOUNDLIST* bl2)
{
    double dx = 0, dy = 0;
    if(bl1->xmin > bl2->xmax)
        dx = bl1->xmin - bl2->xmax;
    else if(bl2->xmin > bl1->xmax)
        dx = bl2->xmin - bl1->xmax;
    if(bl1->ymin > bl2->ymax)
        dy = bl1->ymin - bl2->ymax;
    else if(bl2->ymin > bl1->ymax)
        dy = bl2->ymin - bl1->ymax;
    return dx*dx + dy*dy;
}

double dtw_lb(POSTGIS_POINTARRAY *ps, BOUNDLIST *bl)
{
    double res = 0;
    Trajectory tj(ps);
    for(int i = 0; i < tj.m_points.size(); i++)
    {
        POSTGIS_POINT2D p;
        p.x = tj.m_points[i].m_x;
        p.y = tj.m_points[i].m_y;
        double mindist_square = 1e300;
        for(int j = 0; j < bl->BL_numbox ; j++)
        {
            double dist = bound_b_p_mindist_square(&(bl->BL_data[j]), p);
            if(dist < mindist_square)
                mindist_square = dist;
            if(dist == 0)
                break;
        }
        res += sqrt(mindist_square);
    }
    return res;
}