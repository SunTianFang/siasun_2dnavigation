#include "datasetting.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"

DataSetting::DataSetting()
{

}

DataSetting::~DataSetting()
{

}

std::string DataSetting::get(const std::string &param)
{
    return "";
}

bool DataSetting::get(const std::string &param, std::string &data)
{
    Json::Value root;
    {
        Json::Value item;
        item["name"] = "frontX";
        item["value"] = Json::Value(GData::getObj().front_x);
        root.append(item);
    }
    {
        Json::Value item;
        item["name"] = "frontY";
        item["value"] = Json::Value(GData::getObj().front_y);
        root.append(item);
    }
    {
        Json::Value item;
        item["name"] = "frontTheta";
        item["value"] = Json::Value(GData::getObj().front_theta);
        root.append(item);
    }
//    {
//        Json::Value item;
//        item["name"] = "FB_backward";
//        std::string value = "null";
//        Simulate::getObj().get("setting-param6",value);
//        item["value"] = value;
//        root.append(item);
//    }
    data = root.toStyledString();
    return true;
}
