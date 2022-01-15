//
// Created by Chuang on 2021/10/17.
//

#include <TBlockKey.h>

#include "testFuncs.h"

using std::vector;
int main()
{
    vector<Trajectory> trajs;
    trajs = loadDumpedFiledToTrajs("D://TBlock/dumpedtraj.txt");

    BEnable ena;
    ena.enable[T_box1] = true;
    ena.enable[T_box2] = false;
    ena.enable[T_block1] = false;
    ena.enable[T_block2] = false;
    auto o = OPTcostGlobal(trajs, 100, ena);
//
    ena.enable[T_box1] = false;
    ena.enable[T_box2] = false;
    ena.enable[T_block1] = true;
    ena.enable[T_block2] = true;
    auto s = OPTcostGlobal(trajs, 100, ena);

    ena.enable[T_box1] = true;
    ena.enable[T_box2] = false;
    ena.enable[T_block1] = true;
    ena.enable[T_block2] = false;
    auto p = OPTcostGlobal(trajs, 100, ena);


    for(int i = 0;i<50;i++)
    {
        cout<<i<<"\t"<<o[i]<<"\t"<<s[i]<<"\t"<<p[i]<<"\n";
    }
    return 0;
}