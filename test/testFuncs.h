//
// Created by Chuang on 2021/11/12.
//

#ifndef TBLOCK_TESTFUNCS_H
#define TBLOCK_TESTFUNCS_H

#include <string>
#include <fstream>
#include <set>
#include <map>
#include <sstream>


#define maxLinesToRead 1e10

using namespace std;
struct xyt {
    double x;
    double y;
    double t;
};
template<class Type>
Type stringToNum(const std::string &str) {
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

static vector<Trajectory> loadDumpedFiledToTrajs(string filename) {
    ifstream inFile(filename, ios::in);
    string lineStr;
    set<long> ids;
    vector<Trajectory> res;
    Trajectory tj;
//    tjstat->fromString(lineStr);
    int curLine = 0;
    while (getline(inFile, lineStr) && curLine < 1) {
        try {
            string str;
            stringstream ss(lineStr);
            getline(ss, str);
            long id = stringToNum<long>(str);
            getline(inFile, str);
            tj.loadFromString(str);
            if (tj.m_points.size() >= 2) {
                ids.insert(id);
                res.emplace_back(tj);
                curLine++;
            }
        }
        catch (...) {
            break;
        }
    }

    inFile.close();
    std::cerr<<filename<<endl;
    return res;
}


#endif //TBLOCK_TESTFUNCS_H
