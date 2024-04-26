#ifndef DATAFINISHRECORDDX_H
#define DATAFINISHRECORDDX_H
#include "../dataapi.h"

class DataFinishRecordDx : public DataApi
{
public:
    DataFinishRecordDx();
    virtual ~DataFinishRecordDx();
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // DATAFINISHRECORDDX_H
