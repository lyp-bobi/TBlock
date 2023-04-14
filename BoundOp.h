//
// Created by Chuang on 2023/3/11.
//

#ifndef TBLOCK_BOUNDOP_H
#define TBLOCK_BOUNDOP_H

#include "Bound.h"
#include "postgis.h"

typedef struct POSTGIS_POINT2D{
    double x,y;
}POSTGIS_POINT2D;


extern double bound_b_p_mindist_square(BOUND* b, POSTGIS_POINT2D);

extern double bound_bl_bl_mindist_fast_square(BOUNDLIST* bl1, BOUNDLIST* bl2);

extern double dtw_lb(POSTGIS_POINTARRAY *tj, BOUNDLIST *bl);
#endif //TBLOCK_BOUNDOP_H
