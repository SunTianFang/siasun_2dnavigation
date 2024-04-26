#ifndef DATASTARTRECORDDX_H
#define DATASTARTRECORDDX_H
#include "../dataapi.h"

class DataStartRecordDx : public DataApi
{
public:
    DataStartRecordDx();
    virtual ~DataStartRecordDx();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATASTARTRECORDDX_H
