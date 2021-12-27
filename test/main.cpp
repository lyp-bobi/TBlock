#include <iostream>

#include "TBlockProto.h"
#include <fstream>
#include <sstream>

using namespace std;

string testFileName(){
#if (defined _WIN32 || defined _WIN64 || defined WIN32 || defined WIN64)
    return "D://geolifeCleaner/tdfilter.txt";
#else
    return "/root/tdfilter.txt";
#endif
}

void regress(){
    Trajectory traj;
    ifstream inFile(testFileName(), ios::in);
    string lineStr;
    Trajectory tj;
    TBlockProto tb;
    double time1=0, time2=0;
    int curLine = 0;
    MBR r(116, 116.5, 40, 40.5, 800, 8000);
    int i = 0;
    double time;
    std::chrono::microseconds duration;
    vector<Trajectory> tjs;
    vector<TBlockProto> tbs;
    while (getline(inFile, lineStr)) {
        ++i;
        string str;
        stringstream ss(lineStr);
        getline(ss, str);
        long id = stoll(str);
        getline(inFile, str);
        if(i<5220) continue;
        tj.loadFromString(str);
        tb.cvt_greedy(tj);
        cerr<<i<<endl;
        while(tj.passBy(r)!=tb.passBy(r)){
            cerr<<i<<endl;
            cerr<<tj.toString()<<endl;
            tb.cvt_greedy(tj);
            cerr<<tj.passBy(r);
            cerr<<tb.passBy(r)<<endl;
        }
    }
    inFile.close();
}

int main() {
//    regress();
//    return 0;
    Trajectory traj;
    ifstream inFile(testFileName(), ios::in);
    string lineStr;
    Trajectory tj;
    TBlockProto tb;
    double time1=0, time2=0;
    int curLine = 0;
    MBR r(116, 116.5, 40, 40.5, 6800, 493000);
    int i = 0;
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    double time;
    std::chrono::microseconds duration;
    vector<Trajectory> tjs;
    vector<TBlockProto> tbs;
    while (getline(inFile, lineStr)) {
        ++i;
        string str;
        stringstream ss(lineStr);
        getline(ss, str);
        long id = stoll(str);
        getline(inFile, str);
        tj.loadFromString(str);
        MBR br = tj.getMBR();
        if(br.intersects(r)) {
            tb.cvt_greedy(tj);
            tjs.emplace_back(tj);
            tbs.emplace_back(tb);
        }
        //cout<<tb.m_points.size()<<"\t"<<tb.m_tag<<endl;
    }
    cerr<<"tblock"<<endl;
    start = std::chrono::system_clock::now();
    for(auto &tb:tbs){
        tb.passBy(r);
    }
    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    time = double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    cerr<<time<<endl;


    cerr<<"traj"<<endl;
    start = std::chrono::system_clock::now();
    for(auto &tj:tjs){
        tj.passBy(r);
    }
    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    time = double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    cerr<<time<<endl;
}
