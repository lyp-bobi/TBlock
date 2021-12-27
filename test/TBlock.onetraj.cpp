//
// Created by Chuang on 2021/10/17.
//

#include <TBlockKey.h>

int main()
{
    Trajectory tj;
    std::cout<<"aaa";
    tj.loadFromString("116.467128,39.939488,265079.000000 116.497360,39.961560,265185.000000 116.356050,39.867990,265229.000000 116.360633,39.872935,265379.000000");
    OPTcost(tj);

    tj.loadFromString("1,1,1 2,2,2 4,4,4 8,8,8 3,3,3 10,10,10");
    OPTcost(tj);
    return 0;
}