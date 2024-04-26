#ifndef DATADXLASERPARM_H
#define DATADXLASERPARM_H
#include "../dataapi.h"

class DataDxLaserParm : public DataApi
{
public:
    DataDxLaserParm();
    virtual ~DataDxLaserParm();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATADXLASERPARM_H
