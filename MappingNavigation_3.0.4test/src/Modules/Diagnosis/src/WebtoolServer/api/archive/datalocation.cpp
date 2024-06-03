#include "datalocation.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"

DataLocation::DataLocation()
{

}

DataLocation::~DataLocation()
{

}

std::string DataLocation::get(const std::string &param)
{
    return "";
}

bool DataLocation::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root,item;

    if(GData::getObj().flag){
        item["x"] = Json::Value(GData::getObj().x);
        item["y"] = Json::Value(GData::getObj().y);
        item["theta"] = Json::Value(GData::getObj().theta);
        item["g"] = Json::Value(GData::getObj().uG);
        item["n"] = Json::Value(GData::getObj().uN);
    }
    else{
        return false;
    }

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "post msg : pose info: x = ", GData::getObj().x,", y = ", GData::getObj().y,
                  ", theta = ", GData::getObj().theta,", ug = ", GData::getObj().uG,", uN = ", GData::getObj().uN);
#endif

    root["data"] = item;
    data = root.toStyledString();
    return true;
}
