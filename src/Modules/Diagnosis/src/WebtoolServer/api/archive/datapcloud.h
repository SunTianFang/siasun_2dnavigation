#ifndef DATAPCLOUD_H
#define DATAPCLOUD_H
#include "../dataapi.h"

class DataPCloud : public DataApi
{
public:
    DataPCloud();
    virtual ~DataPCloud();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATACLOUD_H
