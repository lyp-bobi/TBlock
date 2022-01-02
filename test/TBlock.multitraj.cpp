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
    ena.enable[T_block1] = true;
    ena.enable[T_block2] = true;
    auto s = OPTcostGlobal(trajs, 100, ena);
    return 0;
}