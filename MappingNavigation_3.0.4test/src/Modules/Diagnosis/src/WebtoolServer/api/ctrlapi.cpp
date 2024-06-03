#include "ctrlapi.h"
#include <iostream>
#include "json/json.h"
CtrlApi::CtrlApi()
{

}

CtrlApi::~CtrlApi()
{

}

bool CtrlApi::excute(const std::string &cmd)
{
    std::cout << "try excute : " << cmd << std::endl;
    return false;
}

std::string CtrlApi::get(const std::string &param)
{
    return "";
}

bool CtrlApi::get(const std::string &param, std::string &data)
{
    (void)(param);
    Json::Value root;

    root["data"] = Json::Value("ok");

    data = root.toStyledString();
    return true;
}
