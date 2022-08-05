//
// Created by Chuang on 2022/7/3.
//

#include <TBlock_c.h>
#include <TBlockKey.h>
#include <postgres.h>
#include <cfloat>

POINTARRAY *tbox_opt(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {true, false, false, false};
    TBlockKey c = OPTBlock(t, numseg, ena);
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + c.m_size * sizeof(double) * 3);
    res->npoints = c.m_size;
    dlist = (double *) (res->serialized_pointlist);
    for (int i = 0; i < c.m_size; i++) {
        dlist[3 * i] = c.m_pts[i].m_x;
        dlist[3 * i + 1] = c.m_pts[i].m_y;
        dlist[3 * i + 2] = c.m_pts[i].m_t;
    }
    return res;
}

POINTARRAY *tblock_opt(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {false, false, true, true};
    auto c = OPTcost(t, ena, numseg)[numseg];
    if (c.cost > 1e299) {
        c = GreedyPath(t, ena);
    }
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + c.m_route.size() * sizeof(double) * 3);
    res->npoints = +c.m_route.size();
    dlist = (double *) (res->serialized_pointlist);
    int i;
    for (i = 0; i < c.m_route.size() - 1; i++) {
        dlist[3 * i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3 * i + 1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3 * i + 2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3 * i] = t.m_points[c.m_route[i].m_pe.m_plast].m_x;
    dlist[3 * i + 1] = t.m_points[c.m_route[i].m_pe.m_plast].m_y;
    dlist[3 * i + 2] = t.m_points[c.m_route[i].m_pe.m_plast].m_t;
    return res;
}

POINTARRAY *tblock_greedy(POINTARRAY *traj, int numseg) {
    double *dlist = (double *) (traj->serialized_pointlist);
    Trajectory t;
    for (int i = 0; i < traj->npoints; i++) {
        t.m_points.emplace_back(
                Point(dlist[3 * i], dlist[3 * i + 1], dlist[3 * i + 2]));
    }
    BEnable ena = {false, false, true, true};
    auto c = GreedyPathElite(t, ena, numseg);
    POINTARRAY *res = (POINTARRAY *) malloc(
            sizeof(POINTARRAY) + (c.m_route.size() + 1) * sizeof(double) * 3);
    res->npoints = c.m_route.size() + 1;
    dlist = (double *) (res->serialized_pointlist);
    int i = 0;
    for (; i < c.m_route.size(); i++) {
        dlist[3 * i] = t.m_points[c.m_route[i].m_ps.m_plast].m_x;
        dlist[3 * i + 1] = t.m_points[c.m_route[i].m_ps.m_plast].m_y;
        dlist[3 * i + 2] = t.m_points[c.m_route[i].m_ps.m_plast].m_t;
    }
    dlist[3 * i] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_x;
    dlist[3 * i + 1] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_y;
    dlist[3 * i + 2] = t.m_points[c.m_route[i - 1].m_pe.m_plast].m_t;
    return res;
}


#define BOX_MAX_DIM 4


typedef unsigned int BOXFLAGS;

typedef struct _BOXNDF

{

    float xmin, xmax, ymin, ymax, zmin, zmax, tmin, tmax; /* bound of time axis*/

    int srid; /* srid, currently unused */

    BOXFLAGS boxflags; /* boxflags, indicating which axes have meaningful values*/

} BOXNDF;


typedef struct _BOXZF {

    float zmin, zmax;

} BOXZF; /**<@var index key */



typedef struct _BOXTF {

    float tmin, tmax;

} BOXTF;/**<@var index key */


typedef struct _BOX2DF {

    float xmin, xmax, ymin, ymax;

} BOX2DF;/**<@var index key */


typedef struct _BOX2DTF {

    float xmin, xmax, ymin, ymax, tmin, tmax;

} BOX2DTF;/**<@var index key */


typedef struct _BOX3DF {

    float xmin, xmax, ymin, ymax, zmin, zmax;

} BOX3DF;/**<@var index key */


typedef struct _BOX3DTF {

    float xmin, xmax, ymin, ymax, zmin, zmax, tmin, tmax;

} BOX3DTF;/**<@var index key */


#define BOXFLAGS_EMPTY 0 /** <@def default value of boxflags */


/* boxflags manipulating functions*/


#define BOXFLAGS_HASXY 0x01

#define BOXFLAGS_HASZ 0x02

#define BOXFLAGS_HAST 0x04


#define BOXFLAGS_GET_XY(flags) ((bool)(flags)& BOXFLAGS_HASXY) /** <@def check if boxflags has xy */

#define BOXFLAGS_GET_Z(flags) ((bool)(flags)& BOXFLAGS_HASZ) /** <@def check if boxflags has z */

#define BOXFLAGS_GET_T(flags) ((bool)(flags)& BOXFLAGS_HAST) /** <@def check if boxflags has t */

#define BOXFLAGS_TO_CHAR(flags) (char)(flags & 0xff)

#define PG_TS_MODIFIER 1000000.0 /** <@def PG_TS_MODIFIER the convert parameter between Postgres Timestamp to seconds */



/**

* @def TS_COMPRESS_RATE convert epoch(second) to float would yied too much inaccuracy, so we further make it smaller

*

* history:

* Created(yinpei.lyp)

*/

#define TS_COMPRESS_RATE 3600.0


#define TS2FLT_LOW(ts) (gm_float_down( (ts) / PG_TS_MODIFIER/TS_COMPRESS_RATE)) /** <@def TS2FLT_LOW convert timestamp to a float smmaler than it */

#define TS2FLT_HIGH(te) (gm_float_up( (te) / PG_TS_MODIFIER/TS_COMPRESS_RATE)) /** <@def TS2FLT_LOW convert timestamp to a float smmaler than it */

#define FLT2TS(flt) ((flt) * PG_TS_MODIFIER*TS_COMPRESS_RATE) /** <@def convert a float number to Postgres Timestamp */

#define FLT2TS_LOW(flt) (nextafterf(flt, -FLT_MAX) * PG_TS_MODIFIER*TS_COMPRESS_RATE) /** <@def convert a float number to Postgres Timestamp, promise to be lower than origin */

#define FLT2TS_HIGH(flt) (nextafterf(flt, FLT_MAX) * PG_TS_MODIFIER*TS_COMPRESS_RATE) /** <@def convert a float number to Postgres Timestamp, promise to be higher than origin*/


#define TS2EPOCH_LOW(ts) (gm_float_down( (ts) / PG_TS_MODIFIER)) /** <@def TS2EPOCH_LOW convert timestamp to a epoch(float type) smmaler than it */

#define TS2EPOCH_HIGH(te) (gm_float_up( (te) / PG_TS_MODIFIER)) /** <@def TS2EPOCH_LOW convert timestamp to a epoch(float type) smmaler than it */

#define EPOCH2TS(flt) ((flt) * PG_TS_MODIFIER) /** <@def convert epoch(float type) to Postgres Timestamp */



#define FLT2TS_LOW(flt) (nextafterf(flt, -FLT_MAX) * PG_TS_MODIFIER*TS_COMPRESS_RATE) /** <@def convert a float number to Postgres Timestamp, promise to be lower than origin */
#define FLT2TS_HIGH(flt) (nextafterf(flt, FLT_MAX) * PG_TS_MODIFIER*TS_COMPRESS_RATE) /** <@def convert a float number to Postgres Timestamp, promise to be higher than origin*/

/** * @brief multiple bounding box type */
typedef struct _BOXNDF_ARRAY {
    float xmin, xmax, ymin, ymax, zmin, zmax, tmin, tmax; /* bound of time axis*/
    BOXFLAGS boxflags;
    int size;
    BOX3DTF boxes[1];
} BOXNDF_ARRAY;


/** * @brief the binary size of box array */
#define SQLMBOX_VARSIZE(flag, nbox) ((sizeof(int)+sizeof(char)+\
sizeof(float)*((BOXFLAGS_GET_XY(flag)?4:0)+(BOXFLAGS_GET_Z(flag)?2:0) \
+(BOXFLAGS_GET_T(flag)?1:0))*nbox +(BOXFLAGS_GET_T(flag)?sizeof(float):0)) \
+sizeof(float)*((BOXFLAGS_GET_XY(flag)?4:0)+(BOXFLAGS_GET_Z(flag)?2:0) \
+(BOXFLAGS_GET_T(flag)?1:0))*nbox +(BOXFLAGS_GET_T(flag)?sizeof(float):0))

/** * @brief the number of boxes given binary size of box array */
#define SQLMBOX_NUMBOX(flag, nbytes) ((nbytes-sizeof(int)-sizeof(char)-(BOXFLAGS_GET_T(flag)?sizeof(float):0)) /sizeof(float) /((BOXFLAGS_GET_XY(flag)?4:0)+(BOXFLAGS_GET_Z(flag)?2:0)+(BOXFLAGS_GET_T(flag)?1:0)))

bool sqltr_box_array_to_binary(BOXNDF_ARRAY *arr, char **data) {
    char *cursor = NULL;
    int i = 0;
    unsigned int datalen;
    char bflag_short = BOXFLAGS_TO_CHAR(arr->boxflags);
    datalen = SQLMBOX_VARSIZE(arr->boxflags, arr->size);
    *data = (char *) malloc(datalen);
    SET_VARSIZE(*data, datalen);
    cursor = (*data) + sizeof(int);
    memcpy( cursor, &bflag_short, sizeof(char));
    cursor += sizeof(char);
    if (arr->size == 1) {
        arr->boxes[0].
                xmin = arr->xmin;
        arr->boxes[0].
                xmax = arr->xmax;
        arr->boxes[0].
                ymin = arr->ymin;
        arr->boxes[0].
                ymax = arr->ymax;
        arr->boxes[0].
                zmin = arr->zmin;
        arr->boxes[0].
                zmax = arr->zmax;
        arr->boxes[0].
                tmin = arr->tmin;
        arr->boxes[0].
                tmax = arr->tmax;
    }
    if (
            BOXFLAGS_GET_XY(arr->boxflags)) {
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].xmin,  sizeof(float));
            cursor += sizeof(float);
        }
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].xmax,  sizeof(float));
            cursor += sizeof(float);
        }
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].ymin,  sizeof(float));
            cursor += sizeof(float);
        }
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].ymax,  sizeof(float));
            cursor += sizeof(float);
        }
    }
    if (
            BOXFLAGS_GET_Z(arr->boxflags)) {
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].zmin,  sizeof(float));
            cursor += sizeof(float);
        }
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].zmax,  sizeof(float));
            cursor += sizeof(float);
        }
    }
    if (
            BOXFLAGS_GET_T(arr->boxflags)) {
        memcpy( cursor, &arr->boxes[0].tmin,  sizeof(float));
        cursor += sizeof(float);
        for (
                i = 0;
                i <= arr->size - 1; i++) {
            memcpy( cursor, &arr->boxes[i].tmax,  sizeof(float));
            cursor += sizeof(float);
        }
    }
    return 0;
}


bool sqltr_box_array_from_binary(char *data, BOXNDF_ARRAY **arr_ref) {
    char *cursor = NULL;
    int i = 0;
    unsigned int datalen = VARSIZE(data);
    unsigned char bflag_short = *((BOXFLAGS *) (data + sizeof(int)));
    int
    nbox = SQLMBOX_NUMBOX(bflag_short, datalen);
    BOXNDF_ARRAY *arr = NULL;
    *arr_ref = static_cast<BOXNDF_ARRAY *>(malloc(
            sizeof(BOXNDF) + sizeof(BOX3DTF) * (nbox + 1)));
    cursor = data + sizeof(int) + sizeof(char);
    arr = *arr_ref;
    arr->size = nbox;
    arr->boxflags = bflag_short;
    arr->xmin = arr->ymin = arr->zmin = FLT_MAX;
    arr->xmax = arr->ymax = arr->zmax = -FLT_MAX;
    if (BOXFLAGS_GET_XY(bflag_short)) {
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].xmin = *((float *) cursor);
            arr->xmin = std::min(arr->xmin, arr->boxes[i].xmin);
            cursor += sizeof(float);
        }
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].xmax = *((float *) cursor);
            arr->xmax = std::max(arr->xmax, arr->boxes[i].xmax);
            cursor += sizeof(float);
        }
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].ymin = *((float *) cursor);
            arr->ymin = std::min(arr->ymin, arr->boxes[i].ymin);
            cursor += sizeof(float);
        }
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].ymax = *((float *) cursor);
            arr->ymax = std::max(arr->ymax, arr->boxes[i].ymax);
            cursor += sizeof(float);
        }
    } else {
        arr->xmin = arr->ymin = -FLT_MAX;
        arr->xmax = arr->ymax = FLT_MAX;
    }
    if (BOXFLAGS_GET_Z(bflag_short)) {
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].zmin = *((float *) cursor);
            arr->zmin = std::min(arr->zmin, arr->boxes[i].zmin);
            cursor += sizeof(float);
        }
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].zmax = *((float *) cursor);
            arr->zmax = std::max(arr->zmax, arr->boxes[i].zmax);
            cursor += sizeof(float);
        }
    } else {
        arr->zmin = -FLT_MAX;
        arr->zmax = FLT_MAX;
    }
    if (BOXFLAGS_GET_T(bflag_short)) {
        for (i = 0; i < nbox; i++) {
            arr->boxes[i].tmin = *((float *) cursor);
            if (i > 0) { arr->boxes[i - 1].tmax = *((float *) cursor); }
            cursor += sizeof(float);
        }
        arr->boxes[nbox - 1].tmax = *((float *) cursor);
        cursor += sizeof(float);
        arr->tmin = arr->boxes[0].tmin;
        arr->tmax = arr->boxes[nbox - 1].tmax;
    } else {
        arr->tmin = FLT2TS(-FLT_MAX);
        arr->tmax = FLT2TS(FLT_MAX);
    }
    return 0;
}