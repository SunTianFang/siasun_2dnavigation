#include "datadxlaserparm.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"
#include "RawScan.h"
#include "RawMap.h"
#include "RoboManager.h"
#include "RoboLocClnt.h"

DataDxLaserParm::DataDxLaserParm()
{

}

DataDxLaserParm::~DataDxLaserParm()
{

}

std::string DataDxLaserParm::get(const std::string &param)
{
    return "";
}

bool DataDxLaserParm::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root, LaserParmRoot, parmDx;
    Json::Reader Jreader;

    auto pRawMap = RawMapSingleton::GetInstance();
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");

    if(!pRawMap || !FileLaserParm){
        return false;
    }
    int nVersion = 210;
    int SensorCount = 0;
    bool LaserState[4] = {false};
    bool frontState = false;
    bool endState = false;

    unsigned int  nStartTime = 0;
    sensor::CRawScan pFrontScan_;
    if(pRawMap->GetFrontRawScan(pFrontScan_)) {
        nStartTime = static_cast<unsigned int>(pFrontScan_.odom_data.time_stamp - 200);
    }

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            nVersion = LaserParmRoot["version"].asInt();
        }

        //启动时间戳
        //nStartTime

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }
        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }
        }

        if(LaserState[0]){
            frontState = true;
        }
        if(LaserState[1]){
            endState = true;
        }

    }
    FileLaserParm.close();

    parmDx["version"] = Json::Value(nVersion);
    parmDx["startTime"] = Json::Value(nStartTime);
    parmDx["sensorCount"] = Json::Value(SensorCount);
    parmDx["frontState"] = Json::Value(frontState);
    parmDx["endState"] = Json::Value(endState);
    parmDx["frontStartAngle"] = Json::Value(GData::getObj().front_start_angle);
    parmDx["frontEndAngle"] = Json::Value(GData::getObj().front_end_angle);
    parmDx["frontCount"] = Json::Value(GData::getObj().front_laser_count);
    parmDx["frontX"] = Json::Value(GData::getObj().front_x);
    parmDx["frontY"] = Json::Value(GData::getObj().front_y);
    parmDx["frontTheta"] = Json::Value(GData::getObj().front_theta);
    parmDx["frontVisualRangeSize"] = Json::Value(GData::getObj().front_visual_range_size);
    parmDx["frontVisualAngleStart"] = Json::Value(GData::getObj().front_visual_angle_start);
    parmDx["frontVisualAngleEnd"] = Json::Value(GData::getObj().front_visual_angle_end);

    if(SensorCount > 1)
    {
        parmDx["endStartAngle"] = Json::Value(GData::getObj().end_start_angle);
        parmDx["endEndAngle"] = Json::Value(GData::getObj().end_end_angle);
        parmDx["endCount"] = Json::Value(GData::getObj().end_laser_count);
        parmDx["endX"] = Json::Value(GData::getObj().end_x);
        parmDx["endY"] = Json::Value(GData::getObj().end_y);
        parmDx["endTheta"] = Json::Value(GData::getObj().end_theta);
        parmDx["endVisualRangeSize"] = Json::Value(GData::getObj().end_visual_range_size);
        parmDx["endVisualAngleStart"] = Json::Value(GData::getObj().end_visual_angle_start);
        parmDx["endVisualAngleEnd"] = Json::Value(GData::getObj().end_visual_angle_end);
    }

    root["data"] = parmDx;
    data = root.toStyledString();
    return true;
}
