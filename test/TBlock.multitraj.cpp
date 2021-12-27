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
    OPTcostGlobal(trajs, 100);
    return 0;
}