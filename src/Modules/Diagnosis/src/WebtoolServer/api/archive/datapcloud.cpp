#include "datapcloud.h"
#include "json/json.h"
#include "RawScan.h"
#include "HttpCommunicationGlobalData.h"
#include "RawMap.h"
#include "RoboManager.h"
#include "RoboLocClnt.h"

DataPCloud::DataPCloud()
{

}

DataPCloud::~DataPCloud()
{

}

std::string DataPCloud::get(const std::string &param)
{
    return "";
}

bool DataPCloud::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root, cloud, distance, intensity;

    auto pRawMap = RawMapSingleton::GetInstance();
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(!pRoboClnt || !pRawMap){
        return false;
    }
    unsigned char workMode = pRoboClnt->GetCurWorkMode();
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "workMode: ", workMode);
#endif
    if(workMode != RLP_MODE_LOCALIZATION && workMode != RLP_MODE_MAPPING){
        return false;
    }

    if(!pRawMap->GetScanData())
        return false;

    int numPointsFront = GData::getObj().num_points_front;
    int numPointsEnd = GData::getObj().num_points_end;


    for(int i = 0; i < (numPointsFront + numPointsEnd); i++)
    {

        distance.append(Json::Value(GData::getObj().distance[i]));
        intensity.append(Json::Value(GData::getObj().intensity[i]));
    }
    cloud["frontState"] = Json::Value(GData::getObj().front_laser_state);
    cloud["endState"] = Json::Value(GData::getObj().end_laser_state);
    cloud["distance"] = distance;
    cloud["intensity"] = intensity;

    root["data"] = cloud;
    data = root.toStyledString();
    return true;
}
