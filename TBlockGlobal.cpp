//
// Created by Chuang on 2021/11/9.
//

#include "TBlockKey.h"
#include <string>
using namespace std;



//#include <sqlite3.h>
//static sqlite3* db = NULL;
//static string dbfile;
//static sqlite3_stmt *stmt_traj_insert = NULL;
//
//bool db_create_table(){
//    int rc;
//    string sql =
//            "create table if not exists cost(TRAJID INTEGER PRIMARY KEY ASC, LENGTH INTEGER, DATA TEXT)";
//    rc = sqlite3_exec(db,sql.c_str(),NULL,NULL,NULL);
//    if (rc != SQLITE_OK) {
//        cerr << "create table " << sqlite3_errmsg(db) << endl;
//    }
//    return true;
//}
//
//
//bool conn_init(string path){
//    int rc;
//    dbfile = path;
//    if(db==NULL){
//        rc = sqlite3_open(path.c_str(), &db);
//        if (rc != SQLITE_OK) {
//            cerr << "db open failed: " << sqlite3_errmsg(db) << endl;
//        }
//        sqlite3_busy_timeout(db, 30000);
//        db_create_table();
//        string sql;
//        sql ="INSERT OR REPLACE INTO cost(TRAJID, LENGTH, DATA)"
//             " VALUES(?1,?2,?3)";
//        rc = sqlite3_prepare(db,sql.c_str(),-1, &stmt_traj_insert, NULL);
//        if (rc != SQLITE_OK) {
//            cerr << sqlite3_errmsg(db) << endl;
//        }
//
//    }
//    return true;
//}
//

/**
 *
 * @param tjs
 * @param nbox number of boxes to use
 * @return v[i] is the minimum cost using i boxes
 */
vector<double> OPTcostGlobal(vector<Trajectory> &tjs, int nbox, BEnable ena)
{
    /*
     * |one traj| one box| two box|...
     * |two traj| one box| two box|...
     * ...
     */
    vector<vector<double>> cost;
    cost.resize(tjs.size()+1);
    int maxboxes = 0;
    for(int i = 1;i<= tjs.size();i++) // i is the number of trajs
    {
        maxboxes+= tjs[i-1].m_points.size()-1;
        maxboxes = min(maxboxes, nbox);
        cost[i].resize(maxboxes+1);
        for(int j = 0;j<=maxboxes;j++) cost[i][j]=1e300;
    }
    maxboxes = 0;
    for(int i = 1;i<= tjs.size();i++) // i is the number of trajs
    {
        int premaxbox = maxboxes;
        vector<double> curcost = OPTcost(tjs[i-1], ena);
        int curnp = tjs[i-1].m_points.size();
        maxboxes+= curnp-1;
        maxboxes = min(maxboxes, nbox);
        for(int j = i;j<=maxboxes;j++) // j is the number of boxes
        {
            for(int k = max(j - curnp + 1,i-1);k<=min(j-1, premaxbox);k++)//k is the boxes already used
            {
                double value;
                if(k==0)
                    value = curcost[j-k];
                else
                    value = cost[i-1][k] + curcost[j-k];
                cost[i][j] = min(cost[i][j], value);
            }
        }
    }
    return cost.back();
}