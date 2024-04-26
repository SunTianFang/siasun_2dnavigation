#include "datastartrecorddx.h"
#include "iostream"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"
#include "RawScan.h"
#include "RawMap.h"
#include "RoboManager.h"
#include "RoboLocClnt.h"

DataStartRecordDx::DataStartRecordDx()
{

}

DataStartRecordDx::~DataStartRecordDx()
{

}

std::string DataStartRecordDx::get(const std::string &param)
{
    return "";
}

bool DataStartRecordDx::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root;
    if(GData::getObj().startRecordDxFlag){
        return false;
    }

    GData::getObj().startRecordDxFlag = true;

    root["data"] = Json::Value(1);
    data = root.toStyledString();

    return true;
}
