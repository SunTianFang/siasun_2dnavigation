#include "dataapi.h"

DataApi::DataApi()
{

}

DataApi::~DataApi()
{

}

std::string DataApi::get(const std::string& param)
{
    (void)(param);
    return "";
}

bool DataApi::get(const std::string& param, std::string &data)
{
    (void)(param);
    (void)(data);
    return false;
}
