#ifndef DATAPARM_H
#define DATAPARM_H
#include "../dataapi.h"

class DataParm : public DataApi
{
public:
    DataParm();
    virtual ~DataParm();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATAPARM_H
