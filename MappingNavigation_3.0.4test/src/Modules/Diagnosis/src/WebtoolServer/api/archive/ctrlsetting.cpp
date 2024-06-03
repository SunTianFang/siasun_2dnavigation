#include "ctrlsetting.h"
#include "iostream"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"

CtrlSetting::CtrlSetting()
{

}

CtrlSetting::~CtrlSetting()
{

}

bool CtrlSetting::excute(const std::string &cmd)
{
    std::cout << cmd << std::endl;
    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(cmd, root, false))
    {
        return false;
    }
    auto realParam = root["data"];
    auto count = realParam.size();

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "count: ", count, ", call function to set x is : ", realParam[0]["value"].asString().c_str());
#endif
    for (Json::ArrayIndex i = 0; i < count; i++) {
        if(realParam[i]["name"].isString() && realParam[i]["value"].isString())
        {
            //posX, posY, posTheta
            //std::string name = "setting-" + realParam[i]["name"].asString();
            GData::getObj().set(realParam[i]["name"].asString(), realParam[i]["value"].asString());
        }
    }
    return true;
}

std::string CtrlSetting::get(const std::string &param)
{
    return "";
}

bool CtrlSetting::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root;

    root["data"] = Json::Value("ok");

    data = root.toStyledString();
    return true;
}
