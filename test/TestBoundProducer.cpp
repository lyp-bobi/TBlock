//
// Created by Chuang on 2022/9/24.
//
#include <stdlib.h>
#include <TBlockKey.h>
#include <Bound.h>
#include <BoundProducer.h>

int main()
{
    POINTARRAY *arr = NULL;
    Trajectory tj;
    tj.loadFromString("116.467128,39.939488,265079.000000 116.497360,39.961560,265185.000000 116.356050,39.867990,265229.000000 116.360633,39.872935,265379.000000");

    BEnable ena;
    ena.enable[T_block1] = true;
    ena.enable[T_block2] = true;
    arr = tj.asptarray();
    BOUNDPRODUCER prod(arr);
//    prod.produce_tbox_list(1);
    prod.produce_tbox_list(5);
    prod.produce_tbox_list(10);
    prod.produce_tblock_list(1);
    prod.produce_tblock_list(5);
    prod.produce_tblock_list(10);
    return 0;
}