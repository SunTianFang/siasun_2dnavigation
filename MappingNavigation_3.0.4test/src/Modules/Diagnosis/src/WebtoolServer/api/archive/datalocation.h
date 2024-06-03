#ifndef DATALOCATION_H
#define DATALOCATION_H
#include "../dataapi.h"
#include "LocalizeFactory.h"

class DataLocation : public DataApi
{
public:
    DataLocation();
    virtual ~DataLocation();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATALOCATION_H
