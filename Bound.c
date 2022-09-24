//
// Created by Chuang on 2022/9/16.
//
#include <stdlib.h>
#include <Bound.h>
#include <stdbool.h>
#include <string.h>

#define SIZE_BLOCKLIST_PADDING (4) // for pg varsize
#define SIZE_BLOCKLIST_FLAG (1)

char *boolsToBytes(bool *t, int numbox) {
    char *b = malloc((numbox + 7) / 8);
    memset(b, 0, (numbox + 7) / 8);
    for (int i = 0; i < numbox; i++) {
        if(t[i])
            b[i / 8] |= 0x80 >> (i % 8);
    }
    return b;
}

bool *bytesToBools(char *b, int numbox) {
    bool *t = malloc(numbox);
    for (int i = 0; i < numbox; i++) {
        for (int j = 0; j < 8 && 8*i+j < numbox; j++) {
            if (((b[i] << j) & 0x80) == 0x80) {
                t[8 * i + j] = true;
            } else{
                t[8 * i + j] = false;
            }
        }
    }
    return t;
}


unsigned int boundlist_datalen_2d(BOUNDLISTTYPE type, unsigned int numbox) {
    switch (type) {
        case BLT_boxlist:
            return SIZE_BLOCKLIST_PADDING + SIZE_BLOCKLIST_FLAG +
                   numbox * 4 * sizeof(float);
        case BLT_blocklist:
            return SIZE_BLOCKLIST_PADDING + SIZE_BLOCKLIST_FLAG +
                   ((numbox + 7) / 8) + (numbox + 1) * (2 * sizeof(float));
    }
}

unsigned int boundlist_numbox_2d(BOUNDLISTTYPE type, unsigned int datalen) {
    switch (type) {
        case BLT_boxlist:
            return (datalen - SIZE_BLOCKLIST_PADDING - SIZE_BLOCKLIST_FLAG) /
                   4 / sizeof(float);
        case BLT_blocklist:
            return (int) (
                    (datalen - SIZE_BLOCKLIST_PADDING - SIZE_BLOCKLIST_FLAG) /
                    (1.0 / 8 + 2 * sizeof(float))) - 1;
    }
}

BOUNDLIST_BUFFER boundlist_serl_2d(BOUNDLIST *bl) {
    BOUNDLIST_BUFFER buf;
    buf.BLB_size = boundlist_datalen_2d(bl->BL_type, bl->BL_numbox);
    char *res = malloc(buf.BLB_size);
    buf.BLB_data = res;
    res[SIZE_BLOCKLIST_PADDING] = bl->BL_type;
    switch (bl->BL_type) {
        case BLT_boxlist: {
            float *fltdata = (float *) &res[SIZE_BLOCKLIST_PADDING +
                                            SIZE_BLOCKLIST_FLAG];
            int cur = 0;
            for (int i = 0; i < bl->BL_numbox; i++) {
                fltdata[cur++] = bl->BL_data[i].B_data[0];
                fltdata[cur++] = bl->BL_data[i].B_data[1];
                fltdata[cur++] = bl->BL_data[i].B_data[2];
                fltdata[cur++] = bl->BL_data[i].B_data[3];
            }
        }
        break;
        case BLT_blocklist: {
            int len_flag = (bl->BL_numbox + 7) / 8;
            bool *types = (bool *) malloc(sizeof(bool) * bl->BL_numbox);
            float *fltdata = (float *) (res + SIZE_BLOCKLIST_PADDING +
                                        SIZE_BLOCKLIST_FLAG + len_flag);
            int cur = 0;
            for (int i = 0; i < bl->BL_numbox; i++) {
                types[i] = bl->BL_data[i].B_type - BT_block1;
                fltdata[cur++] = bl->BL_data[i].B_data[0];
                fltdata[cur++] = bl->BL_data[i].B_data[1];
            }
            fltdata[cur++] = bl->BL_data[bl->BL_numbox - 1].B_data[2];
            fltdata[cur++] = bl->BL_data[bl->BL_numbox - 1].B_data[3];
            char *type_char = boolsToBytes(types, bl->BL_numbox);
            memcpy(res+SIZE_BLOCKLIST_PADDING+ SIZE_BLOCKLIST_FLAG, type_char, len_flag);
        }
        break;
    }
    return buf;
}

BOUNDLIST *boundlist_deserl_2d(BOUNDLIST_BUFFER in) {
    char *data = in.BLB_data;
    unsigned int datalen = in.BLB_size;
    BOUNDLISTTYPE type = data[SIZE_BLOCKLIST_PADDING];
    int numbox = boundlist_numbox_2d(type, datalen);
    BOUNDLIST *bl = malloc(sizeof(BOUNDLISTTYPE) + sizeof(BOUND) * numbox);
    bl->BL_type = type;
    bl->BL_numbox = numbox;
    switch (type) {
        case BLT_boxlist: {
            float *fltdata = (float*)(
                    data + SIZE_BLOCKLIST_PADDING + SIZE_BLOCKLIST_FLAG);
            int cur = 0;
            for (int i = 0; i < numbox; i++) {
                bl->BL_data[i].B_data[0] = fltdata[cur++];
                bl->BL_data[i].B_data[1] = fltdata[cur++];
                bl->BL_data[i].B_data[2] = fltdata[cur++];
                bl->BL_data[i].B_data[3] = fltdata[cur++];
                bl->BL_data[i].B_type = BT_box1;
            }
        }
        break;
        case BLT_blocklist: {
            int len_flag = (numbox + 7) / 8;
            bool *types = bytesToBools(
                    (data + SIZE_BLOCKLIST_PADDING + SIZE_BLOCKLIST_FLAG),
                    numbox);
            float *fltdata = (float*)(data + SIZE_BLOCKLIST_PADDING + SIZE_BLOCKLIST_FLAG + len_flag);
            int cur = 0;
            for (int i = 0; i < bl->BL_numbox; i++) {
                bl->BL_data[i].B_type = types[i] + BT_block1;
                bl->BL_data[i].B_data[0] = fltdata[cur++];
                bl->BL_data[i].B_data[1] = fltdata[cur++];
                if(i!= 0)
                {
                    bl->BL_data[i-1].B_data[2] = bl->BL_data[i].B_data[0];
                    bl->BL_data[i-1].B_data[3] = bl->BL_data[i].B_data[1];
                }
            }
            bl->BL_data[bl->BL_numbox - 1].B_data[2] = fltdata[cur++];
            bl->BL_data[bl->BL_numbox - 1].B_data[3] = fltdata[cur++];
        }
        break;
    }
    return bl;
}