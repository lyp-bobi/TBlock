//
// Created by Chuang on 2022/9/16.
//

#ifndef TBLOCK_TBLOCK_H
#define TBLOCK_TBLOCK_H

#include <stdbool.h>

// type of boxes
//
typedef enum BOUNDTYPE{
    BT_box1 = 0, //normal box
    BT_box2, //slant box
    BT_block1,//normal block
    BT_block2,//slant block
    BT_end
}BOUNDTYPE;

// one bounding structure
typedef struct BOUND
{
    BOUNDTYPE B_type;
#define BOUND_MAX_FLOAT_LENGTH 6
    float B_data[BOUND_MAX_FLOAT_LENGTH];
}BOUND;

// different types of bounds
typedef struct BOUND_BOX_2D
{
    BOUNDTYPE B_type;
    float xmin, ymin, xmax, ymax;
}BOUND_BOX_2D;

typedef struct BOUND_BLOCK1_2D
{
    BOUNDTYPE B_type;
    float xs, ys, xe, ye;
}BOUND_BLOCK1_2D;

typedef struct BOUND_BLOCK2_2D
{
    BOUNDTYPE B_type;
    float xs, ys, xe, ye;
}BOUND_BLOCK2_2D;

// type of boxlist
typedef enum BOUNDLISTTYPE{
    BLT_boxlist,
    BLT_blocklist
}BOUNDLISTTYPE;

// a list of bounding structures bounding a linestring/trajectory
typedef struct BOUNDLIST
{
    BOUNDLISTTYPE BL_type; //type
    unsigned int BL_numbox;
    float xmin, xmax, ymin, ymax;
    struct BOUND BL_data[1]; //bounds
}BOUNDLIST;

typedef struct BOUNDLIST_SERL
{
    unsigned int BLB_size;
    char* BLB_data;
}BOUNDLIST_SERL;

#ifdef __cplusplus
extern "C"
{
#endif
extern unsigned int boundlist_datalen_2d(BOUNDLISTTYPE type,unsigned int numbox);
extern unsigned int boundlist_numbox_2d(BOUNDLISTTYPE type, unsigned int datalen);
extern BOUNDLIST_SERL boundlist_serl_2d(BOUNDLIST* bl);
extern BOUNDLIST* boundlist_deserl_2d(BOUNDLIST_SERL in);
extern bool intersects_b_b(const BOUND* a, const BOUND *b);
extern bool intersects_bl_b(const BOUNDLIST* a, const BOUND *b);
#ifdef __cplusplus
}
#endif

#endif //TBLOCK_TBLOCK_H
