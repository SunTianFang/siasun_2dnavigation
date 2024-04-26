#ifndef DATADXSCANDATA_H
#define DATADXSCANDATA_H
#include "../dataapi.h"

class DataDxScanData : public DataApi
{
public:
    DataDxScanData();
    virtual ~DataDxScanData();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATADXSCANDATA_H
