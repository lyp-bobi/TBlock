//
// Created by Chuang on 2022/7/3.
//

#ifndef TBLOCK_TBLOCK_C_H
#define TBLOCK_TBLOCK_C_H

#include <cstdint>

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

struct Point_c{
    double x,y,z;
};

struct TBlock_c{
public:
    int m_size;
    BType *m_type;
    
};

struct TBOX_SERL{
    uint32_t varsize;
    float coords[0];
};

struct TBLOCK_SERL{
    uint32_t varsize;
    char data[0];
};

#define TBOX_NBOX_TO_VARSIZE(n) (sizeof(uint32) + sizeof(float8) * 6 * n)

#define TBOX_VARSIZE_TO_NBOX(len) (((len) - sizeof(uint32))/6/sizeof(float8))

#define TBLOCK_NBOX_TO_VARSIZE(n) (sizeof(uint32) + (1 + sizeof(float8) * 6) * n)

#define TBLOCK_VARSIZE_TO_NBOX(len) (((len) - sizeof(uint32))/6/(1+sizeof(float8)))

//extern "C"{
//
//    //POINTARRAY** tbox_opt_multi(POINTARRAY **traj, int numseg);
//
//    varlena* tbox_opt_to_binary(POINTARRAY *traj, int numseg);
//    varlena* tblock_opt_to_binary(POINTARRAY *traj, int numseg);
//    varlena* tblock_greedy_to_binary(POINTARRAY *traj, int numseg);
//
//    TBlock_c binary_to_tbox(varlena* data);
//    TBlock_c binary_to_tblock(varlena* data);
//};


#endif //TBLOCK_TBLOCK_C_H
