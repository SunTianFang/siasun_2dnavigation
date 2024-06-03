#pragma once


#include <string>
#include "type.h"


using namespace std;

    void SignalHandlerRegister_test();

void Navi_Init(LaserNaviConfig config);
void Navi_HandleLaserData(laserscan_msg *msg);
void Navi_HandleEncoderData(odometry_msg *msg);

bool Navi_CreateMap(void);
int Navi_SaveMap(const string filename);

bool Navi_StopMapping(void);

void Navi_AddLaserID(std::string laser_id);


void Navi_RegisterSlamCallBack(SlamResultCbFunc pFunc);

void Navi_RunFinalOpt();


void Navi_GetSubmapList(submap_list &submaps);
void Navi_GetSubmapData(int submap_id,submap_data *pdata);
void Navi_GetNodeData(std::vector<node_data> &nodeDatas);
map<int, vector<int>> Navi_GetPossibleConstraintPairs();
