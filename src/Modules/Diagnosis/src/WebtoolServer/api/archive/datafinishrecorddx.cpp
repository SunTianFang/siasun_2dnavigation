#include "datafinishrecorddx.h"
#include "iostream"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"
#include "RawScan.h"
#include "RawMap.h"
#include "RoboManager.h"
#include "RoboLocClnt.h"


DataFinishRecordDx::DataFinishRecordDx()
{

}

DataFinishRecordDx::~DataFinishRecordDx()
{

}

std::string DataFinishRecordDx::get(const std::string &param)
{
    return "";
}

bool DataFinishRecordDx::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root;
    if(!GData::getObj().startRecordDxFlag){
        return false;
    }

    GData::getObj().finishRecordDxFlag = true;
    GData::getObj().startRecordDxFlag  = false;

    root["data"] = Json::Value(1);
    data = root.toStyledString();

    return true;
}
