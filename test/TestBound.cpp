//
// Created by Chuang on 2022/9/24.
//
#include <stdlib.h>
#include <Bound.h>

int main()
{
    BOUNDLIST* a, *b;
    a = static_cast<BOUNDLIST *>(malloc(
            sizeof(BOUNDLIST) + 10 * sizeof(BOUND)));
    a->BL_type = BLT_boxlist;
    a->BL_numbox = 2;
    a->BL_data[0].B_type = BT_box1;
    a->BL_data[0].B_data[0] = 1;
    a->BL_data[0].B_data[1] = 1;
    a->BL_data[0].B_data[2] = 2;
    a->BL_data[0].B_data[3] = 2;
    a->BL_data[1].B_type = BT_box1;
    a->BL_data[1].B_data[0] = 2;
    a->BL_data[1].B_data[1] = 2;
    a->BL_data[1].B_data[2] = 4;
    a->BL_data[1].B_data[3] = 4;
    BOUNDLIST_SERL buf = boundlist_serl_2d(a);
    b = boundlist_deserl_2d(buf);
    a->BL_type = BLT_blocklist;
    a->BL_data[0].B_type = BT_block1;
    a->BL_data[1].B_type = BT_block2;
    buf = boundlist_serl_2d(a);
    b = boundlist_deserl_2d(buf);
    return 0;
}