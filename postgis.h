//
// Created by Chuang on 2022/10/3.
//

#ifndef TBLOCK_POSTGIS_H
#define TBLOCK_POSTGIS_H
#define LWFLAG_Z        0x01
#define LWFLAG_M        0x02
#define LWFLAG_BBOX     0x04
#define LWFLAG_GEODETIC 0x08
#define LWFLAG_READONLY 0x10
#define LWFLAG_SOLID    0x20
#define FLAGS_GET_Z(flags)         ((flags) & LWFLAG_Z)
#define FLAGS_GET_M(flags)        (((flags) & LWFLAG_M)>>1)
#define FLAGS_GET_BBOX(flags)     (((flags) & LWFLAG_BBOX)>>2)
#define FLAGS_GET_GEODETIC(flags) (((flags) & LWFLAG_GEODETIC)>>3)
#define FLAGS_GET_READONLY(flags) (((flags) & LWFLAG_READONLY)>>4)
#define FLAGS_GET_SOLID(flags)    (((flags) & LWFLAG_SOLID)>>5)
#define FLAGS_NDIMS(flags) (2 + FLAGS_GET_Z(flags) + FLAGS_GET_M(flags))
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
POSTGIS_POINTARRAY;
#endif //TBLOCK_POSTGIS_H
