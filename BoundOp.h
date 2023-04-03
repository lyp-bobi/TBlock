//
// Created by Chuang on 2023/3/11.
//

#ifndef TBLOCK_BOUNDOP_H
#define TBLOCK_BOUNDOP_H

#include <Bound.h>
#include "postgis.h"

typedef struct POINT2D{
    double x,y;
}POINT2D;


extern double bound_b_p_mindist_square(BOUND* b, POINT2D);

extern double dtw_lb(POSTGIS_POINTARRAY *tj, BOUNDLIST *bl);
#endif //TBLOCK_BOUNDOP_H
