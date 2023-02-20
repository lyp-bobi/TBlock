//
// Created by Chuang on 2022/9/16.
//
#include <stdlib.h>
#include <Bound.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>
#include <math.h>

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
            return SIZE_BLOCKLIST_FLAG +
                   numbox * 4 * sizeof(float);
        case BLT_blocklist:
            return SIZE_BLOCKLIST_FLAG +
                   ((numbox + 7) / 8) + (numbox + 1) * (2 * sizeof(float));
    }
}

unsigned int boundlist_numbox_2d(BOUNDLISTTYPE type, unsigned int datalen) {
    switch (type) {
        case BLT_boxlist:
            return (datalen - SIZE_BLOCKLIST_FLAG) /
                   4 / sizeof(float);
        case BLT_blocklist:
            return (int) (
                    (datalen - SIZE_BLOCKLIST_FLAG) /
                    (1.0 / 8 + 2 * sizeof(float))) - 1;
    }
}

BOUNDLIST_SERL boundlist_serl_2d(BOUNDLIST *bl) {
    BOUNDLIST_SERL buf;
    buf.BLB_size = boundlist_datalen_2d(bl->BL_type, bl->BL_numbox);
    char *res = malloc(buf.BLB_size);
    buf.BLB_data = res;
    res[0] = bl->BL_type;
    switch (bl->BL_type) {
        case BLT_boxlist: {
            float *fltdata = (float *) &res[SIZE_BLOCKLIST_FLAG];
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
            float *fltdata = (float *) (res + SIZE_BLOCKLIST_FLAG + len_flag);
            int cur = 0;
            for (int i = 0; i < bl->BL_numbox; i++) {
                types[i] = bl->BL_data[i].B_type - BT_block1;
                fltdata[cur++] = bl->BL_data[i].B_data[0];
                fltdata[cur++] = bl->BL_data[i].B_data[1];
            }
            fltdata[cur++] = bl->BL_data[bl->BL_numbox - 1].B_data[2];
            fltdata[cur++] = bl->BL_data[bl->BL_numbox - 1].B_data[3];
            char *type_char = boolsToBytes(types, bl->BL_numbox);
            memcpy(res + SIZE_BLOCKLIST_FLAG, type_char, len_flag);
			free(types);
			free(type_char);
        }
        break;
    }
    return buf;
}
#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))
BOUNDLIST *boundlist_deserl_2d(BOUNDLIST_SERL in) {
    char *data = in.BLB_data;
    unsigned int datalen = in.BLB_size;
    BOUNDLISTTYPE type = data[0];
    int numbox = boundlist_numbox_2d(type, datalen);
    BOUNDLIST *bl = malloc(sizeof(BOUNDLIST) + sizeof(BOUND) * numbox);
    bl->BL_type = type;
    bl->BL_numbox = numbox;
    bl->xmin = bl->ymin = FLT_MAX;
    bl->xmax = bl->ymax = - FLT_MAX;
    switch (type) {
        case BLT_boxlist: {
            float *fltdata = (float*)(
                    data + SIZE_BLOCKLIST_FLAG);
            int cur = 0;
            for (int i = 0; i < numbox; i++) {
                bl->BL_data[i].B_data[0] = fltdata[cur++];
                bl->BL_data[i].B_data[1] = fltdata[cur++];
                bl->BL_data[i].B_data[2] = fltdata[cur++];
                bl->BL_data[i].B_data[3] = fltdata[cur++];
                bl->BL_data[i].B_type = BT_box1;
                if(bl->xmin > bl->BL_data[i].B_data[0])
                {
                    bl->xmin = bl->BL_data[i].B_data[0];
                }
                if(bl->ymin > bl->BL_data[i].B_data[1])
                {
                    bl->ymin = bl->BL_data[i].B_data[1];
                }
	            if(bl->xmax < bl->BL_data[i].B_data[2])
	            {
		            bl->xmax = bl->BL_data[i].B_data[2];
	            }
                if(bl->ymax < bl->BL_data[i].B_data[3])
                {
                    bl->ymax = bl->BL_data[i].B_data[3];
                }
            }
        }
        break;
        case BLT_blocklist: {
            int len_flag = (numbox + 7) / 8;
            bool *types = bytesToBools(
                    (data + SIZE_BLOCKLIST_FLAG),
                    numbox);
            float *fltdata = (float*)(data + SIZE_BLOCKLIST_FLAG + len_flag);
            int cur = 0;
            for (int i = 0; i < bl->BL_numbox; i++) {
                bl->BL_data[i].B_type = types[i] + BT_block1;
                bl->BL_data[i].B_data[0] = fltdata[cur++];
                bl->BL_data[i].B_data[1] = fltdata[cur++];
                if(i != 0)
                {
                    bl->BL_data[i-1].B_data[2] = bl->BL_data[i].B_data[0];
                    bl->BL_data[i-1].B_data[3] = bl->BL_data[i].B_data[1];
                }
            }
            bl->BL_data[bl->BL_numbox - 1].B_data[2] = fltdata[cur++];
            bl->BL_data[bl->BL_numbox - 1].B_data[3] = fltdata[cur++];
            for(int i = 0; i< bl->BL_numbox;i++)
            {
                if(bl->BL_data[i].B_type == BT_block1)
                {
                    if(bl->xmin > bl->BL_data[i].B_data[0])
                    {
                        bl->xmin = bl->BL_data[i].B_data[0];
                    }
                    if(bl->ymin > bl->BL_data[i].B_data[1])
                    {
                        bl->ymin = bl->BL_data[i].B_data[1];
                    }
                    if(bl->xmax < bl->BL_data[i].B_data[2])
                    {
                        bl->xmax = bl->BL_data[i].B_data[2];
                    }
                    if(bl->ymax < bl->BL_data[i].B_data[3])
                    {
                        bl->ymax = bl->BL_data[i].B_data[3];
                    }
                }
                else if(bl->BL_data[i].B_type == BT_block2)
                {
                    BOUND_BLOCK2_2D *b1 = &(bl->BL_data[i]);
                    double sigma2 = (b1->xs + b1->xe + b1->ys + b1->ye) / 2;
                    double xmin = min(min(b1->xs, b1->xe),
                                      min(sigma2 - b1->ys, sigma2 - b1->ye)),
                            xmax = max(max(b1->xs, b1->xe),
                                       max(sigma2 - b1->ys, sigma2 - b1->ye)),
                            ymin = min(min(b1->ys, b1->ye),
                                       min(sigma2 - b1->xs, sigma2 - b1->xe)),
                            ymax = max(max(b1->ys, b1->ye),
                                       max(sigma2 - b1->xs, sigma2 - b1->xe));
                    if(bl->xmin > xmin)
                    {
                        bl->xmin = xmin;
                    }
                    if(bl->ymin > ymin)
                    {
                        bl->ymin = ymin;
                    }
                    if(bl->xmax < xmax)
                    {
                        bl->xmax = xmax;
                    }
                    if(bl->ymax < ymax)
                    {
                        bl->ymax = ymax;
                    }
                }
            }
        }
        break;
    }
    return bl;
}

bool intersects_b_b(const BOUND* a, const BOUND *b)
{
    if(a->B_type == BT_box1 && b->B_type == BT_box1)
    {
        BOUND_BOX_2D *b1 = (BOUND_BOX_2D*)a, *b2 = (BOUND_BOX_2D*)b;
        if(b1->xmin>b2->xmax || b1->xmax<b2->xmin || b1->ymin>b2->ymax || b1->ymax<b2->ymin)
            return false;
        return true;
    }
    else if(a->B_type == BT_block1 && b->B_type == BT_box1)
    {
        BOUND_BLOCK1_2D *b1 = (BOUND_BLOCK1_2D*)a;
        BOUND_BOX_2D *b2 = (BOUND_BOX_2D*)b;
        double xmin = min(b1->xs,b1->xe), xmax = max(b1->xs,b1->xe),
                ymin = min(b1->ys,b1->ye), ymax = max(b1->ys,b1->ye);
        if(xmin>b2->xmax || xmax<b2->xmin || ymin>b2->ymax || ymax<b2->ymin)
            return false;
        return true;
    }
    else if(a->B_type == BT_block2 && b->B_type == BT_box1)
    {
        BOUND_BLOCK2_2D *b1 = (BOUND_BLOCK2_2D*)a;
        BOUND_BOX_2D *b2 = (BOUND_BOX_2D*)b;
        {
            double sigma2 = (b1->xs + b1->xe + b1->ys + b1->ye) / 2;
            double xmin = min(min(b1->xs, b1->xe),
                              min(sigma2 - b1->ys, sigma2 - b1->ye)),
                    xmax = max(max(b1->xs, b1->xe),
                               max(sigma2 - b1->ys, sigma2 - b1->ye)),
                    ymin = min(min(b1->ys, b1->ye),
                               min(sigma2 - b1->xs, sigma2 - b1->xe)),
                    ymax = max(max(b1->ys, b1->ye),
                               max(sigma2 - b1->xs, sigma2 - b1->xe));
            if (xmin > b2->xmax || xmax < b2->xmin || ymin > b2->ymax ||
                ymax < b2->ymin)
                return false;
        }
        {
            double us = b1->xs + b1->ys, vs = b1->xs - b1->ys,
                    ue = b1->xe + b1->ye, ve = b1->xe - b1->ye;
            double umin1 = min(us, ue), umax1 = max(us, ue),
                vmin1 = min(vs, ve), vmax1 = max(vs, ve);
            double umin2 = b2->xmin+b2->ymin, umax2 = b2->xmax + b2->ymax,
                vmin2 = b2->xmin - b2->ymax, vmax2 = b2->xmax - b2->ymin;
            if (umin1 > umax2 || umin2 > umax1 || vmin1 > vmax2 || vmin2 > vmax1)
                return false;
        }
        return true;
    }
    else if(b->B_type == BT_block2 && a->B_type == BT_box1)
    {
        return intersects_b_b(b,a);
    } else
    {
        return false;
    }
}

bool intersects_bl_b(const BOUNDLIST* a, const BOUND *b)
{
    int i;
    for(i = 0; i < a->BL_numbox;i++) {
        if (intersects_b_b(&a->BL_data[i], b))
        {
            return true;
        }
    }
    return false;
}
