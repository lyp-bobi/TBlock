//
// Created by Chuang on 2023/3/21.
//
extern "C"
{
#include <Bound.h>
#include <BoundOp.h>
#include <float.h>
}
#include <Trajectory.hpp>

#define float_down(f)   nextafterf(nextafterf((f),-FLT_MAX),-FLT_MAX)
#define float_up(f)   nextafterf(nextafterf((f),FLT_MAX),FLT_MAX)

double bound_b_p_mindist_square(BOUND* b, POSTGIS_POINT2D p)
{
    if(b->B_type == BT_box1)
    {
        double ret = 0;
        BOUND_BOX_2D *b1 = (BOUND_BOX_2D*)b;
        double xmin =float_down(b1->xmin), xmax =float_up(b1->xmax),
                ymin =float_down(b1->ymin), ymax =float_up(b1->ymax);
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
        double xmin =float_down(std::min(b1->xs,b1->xe)), xmax =float_up(std::max(b1->xs,b1->xe)),
                ymin =float_down(std::min(b1->ys,b1->ye)), ymax =float_up(std::max(b1->ys,b1->ye));
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
        double umin =float_down(std::min(us, ue)), umax =float_up(std::max(us, ue)),
                vmin =float_down(std::min(vs, ve)), vmax =float_up(std::max(vs, ve));
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

#include <BoundProducer.hpp>

bool use_tblock = true;

BOUNDLIST* ptarray_to_boundlist(POSTGIS_POINTARRAY *ptarr)
{
    BOUNDLIST *ret;
    BOUNDPRODUCER prod(ptarr);
    if(!use_tblock)
    {
        ret = prod.produce_tbox_list(8);
    }
    else
    {
        ret = prod.produce_tblock_list(8);
    }
    return ret;
}

BOUNDLIST* ptarray_to_boundlist_maxsize(POSTGIS_POINTARRAY *ptarr, int maxbox)
{
    BOUNDLIST *ret;
    BOUNDPRODUCER prod(ptarr);
    if(!use_tblock)
    {
        ret = prod.produce_tbox_list(8);
    }
    else
    {
        ret = prod.produce_tblock_list(8);
        if (ret->BL_numbox > maxbox)
        {
            free(ret);
            ret = prod.produce_tbox_list(8);
        }
    }
    return ret;
}


static bool intersects_b_line(double x1, double y1, double x2, double y2, const BOUND *b)
{
    if(b->B_type == BT_box1 || b->B_type == BT_block1)
    {
        BOUND_BOX_2D *b2 = (BOUND_BOX_2D*)b;
        MBR r(b2->xmin, b2->xmax, b2->ymin, b2->ymax, -FLT_MAX, FLT_MAX);
        Point p1(x1,y1,0), p2(x2,y2,0);
        return r.intersects(p1,p2);
    }
    else if(b->B_type == BT_block2)
    {
        BOUND_BLOCK2_2D *b1 = (BOUND_BLOCK2_2D*)b;
        {
            double u1 = x1 + y1, v1 = x1 - y1,
                u2 = x2 + y2, v2 = x2 - y2;

            double us = b1->xs + b1->ys, vs = b1->xs - b1->ys,
                    ue = b1->xe + b1->ye, ve = b1->xe - b1->ye;
            double umin1 = std::min(us, ue), umax1 = std::max(us, ue),
                    vmin1 = std::min(vs, ve), vmax1 = std::max(vs, ve);
            MBR r(umin1, umax1, vmin1, vmax1, -FLT_MAX, FLT_MAX);
            Point p1(u1, v1, 0), p2(u2, v2, 0);
            return r.intersects(p1, p2);
        }
    }
    else
    {
        return false;
    }
}

bool intersects_bl_ptarr(POSTGIS_POINTARRAY *ptarr, const BOUNDLIST *b)
{
    double *dlist = (double *) (ptarr->serialized_pointlist);
    Trajectory t;
    int dim = FLAGS_NDIMS(ptarr->flags);
    int i;
    for (i = 0; i < ptarr->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[dim * i], dlist[dim * i + 1], i));
    }
    MBR r = t.getMBR();
    BOUND_BOX_2D r2d;
    r2d.B_type = BT_box1;
    r2d.xmin = r.m_xmin;
    r2d.xmax = r.m_xmax;
    r2d.ymin = r.m_ymin;
    r2d.ymax = r.m_ymax;
    for (i = 0; i< b->BL_numbox; i++)
    {
        if (intersects_bl_b(b, (BOUND*)&r2d))
        {
            for (int j = 0; j < t.m_points.size() - 1;j++)
            {
                if (intersects_b_line(t.m_points[j].m_x, t.m_points[j].m_y,
                                      t.m_points[j+1].m_x, t.m_points[j+1].m_y, &b->BL_data[i]))
                {
                    return true;
                }
            }
        }
    }
    return false;
}