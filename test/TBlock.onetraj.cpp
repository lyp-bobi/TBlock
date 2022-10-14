//
// Created by Chuang on 2021/10/17.
//

#include <TBlockKey.hpp>

int main()
{
    Trajectory tj;
    tj.loadFromString("116.467128,39.939488,265079.000000 116.497360,39.961560,265185.000000 116.356050,39.867990,265229.000000 116.360633,39.872935,265379.000000");

    BEnable ena;
    ena.enable[T_block1] = true;
    ena.enable[T_block2] = true;
//    auto a = OPTcost(tj, ena);
//    std::cout<<a.back().toString()<<std::endl;

//    tj.loadFromString("1,1,1 4,4,2 4,4,4 8,8,8 9,9,9 10,10,10");
    auto c = OPTcost(tj,ena);
    auto d = GreedyPath(tj,ena);
    auto e = GreedyPathElite(tj,ena, 5);
    std::cout<<c[2].toString()<<std::endl;
    return 0;
}