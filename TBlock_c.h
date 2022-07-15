//
// Created by Chuang on 2022/7/3.
//

#ifndef TBLOCK_TBLOCK_C_H
#define TBLOCK_TBLOCK_C_H

#include <cstdint>

/*  POINTARRAY
        *  Point array abstracts a lot of the complexity of points and point lists.
*  It handles 2d/3d translation
*    (2d points converted to 3d will have z=0 or NaN)
*  DO NOT MIX 2D and 3D POINTS! EVERYTHING* is either one or the other
*/
typedef struct
{
    uint32_t npoints;   /* how many points we are currently storing */
    uint32_t maxpoints; /* how many points we have space for in serialized_pointlist */

    /* Use FLAGS_* macros to handle */
    uint16_t flags;

    /* Array of POINT 2D, 3D or 4D, possibly misaligned. */
    uint8_t *serialized_pointlist;
}
POINTARRAY;
enum BType{
    T_box1 = 0,
    T_box2,
    T_block1,
    T_block2,
    T_end
};

struct BEnable{
    bool enable[T_end]={false, false, false, false};
};

extern "C"{
    POINTARRAY** tbox_opt_multi(POINTARRAY **traj, int numseg);
    POINTARRAY* tbox_opt(POINTARRAY *traj, int numseg);
    POINTARRAY* tblock_opt(POINTARRAY *traj, int numseg);
    POINTARRAY* tblock_greedy(POINTARRAY *traj, int numseg);
};


#endif //TBLOCK_TBLOCK_C_H
