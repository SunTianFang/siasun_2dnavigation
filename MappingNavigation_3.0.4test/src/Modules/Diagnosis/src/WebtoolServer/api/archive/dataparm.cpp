#include "dataparm.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"

DataParm::DataParm()
{

}

DataParm::~DataParm()
{

}

std::string DataParm::get(const std::string &param)
{
    return "";
}

bool DataParm::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root, parm, LaserParmRoot;
    Json::Reader Jreader;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    if(!FileLaserParm){
        return false;
    }
    int SensorCount = 0;
    int RealSensorCount = 0;
    bool LaserState[4] = {false};
    bool frontState = false;
    bool endState = false;

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }
        RealSensorCount = SensorCount;
        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }

            if(!LaserState[j])
                RealSensorCount--;
        }

        if(LaserState[0]){
            frontState = true;
        }
        if(LaserState[1]){
            endState = true;
        }

    }

    parm["num"] = Json::Value(RealSensorCount);
    parm["frontX"] = Json::Value(GData::getObj().front_x);
    parm["frontY"] = Json::Value(GData::getObj().front_y);
    parm["frontTheta"] = Json::Value(GData::getObj().front_theta);
    parm["frontLaserCount"] = Json::Value(GData::getObj().front_laser_count);
    parm["frontStartAngle"] = Json::Value(GData::getObj().front_start_angle);
    parm["frontEndAngle"] = Json::Value(GData::getObj().front_end_angle);
    parm["frontState"] = Json::Value(frontState);

    if(GData::getObj().laser_num > 1){
        parm["endX"] = Json::Value(GData::getObj().end_x);
        parm["endY"] = Json::Value(GData::getObj().end_y);
        parm["endTheta"] = Json::Value(GData::getObj().end_theta);
        parm["endLaserCount"] = Json::Value(GData::getObj().end_laser_count);
        parm["endStartAngle"] = Json::Value(GData::getObj().end_start_angle);
        parm["endEndAngle"] = Json::Value(GData::getObj().end_end_angle);
        parm["endState"] = Json::Value(endState);
    }

    parm["ug"] = Json::Value(GData::getObj().evaluate_uG);
    parm["un"] = Json::Value(GData::getObj().evaluate_uN);

    root["data"] = parm;
    data = root.toStyledString();
    return true;
}
