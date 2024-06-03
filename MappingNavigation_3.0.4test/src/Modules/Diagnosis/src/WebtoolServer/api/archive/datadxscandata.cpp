#include "datadxscandata.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"
#include "RawScan.h"
#include "RawMap.h"
#include "RoboManager.h"
#include "RoboLocClnt.h"

DataDxScanData::DataDxScanData()
{

}

DataDxScanData::~DataDxScanData()
{

}

std::string DataDxScanData::get(const std::string &param)
{
    return "";
}

bool DataDxScanData::get(const std::string &param, std::string &data)
{
    std::cout << "By Sam: Get scanData!!!!!!!!" << std::endl;

    (void)(param);
    Json::Value root, scanData, distance, intensity;

    auto pRawMap = RawMapSingleton::GetInstance();
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(!pRoboClnt || !pRawMap){
        std::cout << "false 1" << std::endl;
        return false;
    }
    unsigned char workMode = pRoboClnt->GetCurWorkMode();
    if(workMode != RLP_MODE_LOCALIZATION && workMode != RLP_MODE_MAPPING){
        std::cout << "false 2" << std::endl;
        return false;
    }
    if(!pRawMap->GetScanData()){
        std::cout << "false 3" << std::endl;
        return false;
    }

    int numPointsFront = GData::getObj().num_points_front;
    int numPointsEnd = GData::getObj().num_points_end;

    for(int i = 0; i < (numPointsFront + numPointsEnd); i++)
    {

        distance.append(Json::Value(GData::getObj().distance[i]));
        intensity.append(Json::Value(GData::getObj().intensity[i]));
    }

    std::cout << "By Sam: laser points = " << numPointsFront + numPointsEnd << std::endl;

    scanData["odomFlag"] = Json::Value(GData::getObj().odomFlag);
    scanData["odomTimeStamp"] = Json::Value(GData::getObj().odomTimeStamp);
    scanData["velX"] = Json::Value(GData::getObj().velX);
    scanData["velY"] = Json::Value(GData::getObj().velY);
    scanData["velTheta"] = Json::Value(GData::getObj().velTheta);
    scanData["localOdomX"] = Json::Value(GData::getObj().localOdomX);
    scanData["localOdomY"] = Json::Value(GData::getObj().localOdomY);
    scanData["localOdomTheta"] = Json::Value(GData::getObj().localOdomTheta);
    scanData["globalOdomX"] = Json::Value(GData::getObj().globalOdomX);
    scanData["globalOdomY"] = Json::Value(GData::getObj().globalOdomY);
    scanData["globalOdomTheta"] = Json::Value(GData::getObj().globalOdomTheta);
    scanData["laserTimeStamp"] = Json::Value(GData::getObj().laserTimeStamp);
    scanData["frontState"] = Json::Value(GData::getObj().front_laser_state);
    scanData["endState"] = Json::Value(GData::getObj().end_laser_state);
    scanData["laserDistance"] = distance;
    scanData["laserIntensity"] = intensity;

    std::cout << "By Sam: laser front = " << GData::getObj().front_laser_state << std::endl;
    std::cout << "By Sam: laser end = " << GData::getObj().end_laser_state << std::endl;

    root["data"] = scanData;
    data = root.toStyledString();

    return true;
}
