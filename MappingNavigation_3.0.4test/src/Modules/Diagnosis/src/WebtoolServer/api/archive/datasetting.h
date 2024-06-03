#ifndef DATASETTING_H
#define DATASETTING_H
#include "../dataapi.h"

class DataSetting : public DataApi
{
public:
    DataSetting();
    virtual ~DataSetting();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATASETTING_H
