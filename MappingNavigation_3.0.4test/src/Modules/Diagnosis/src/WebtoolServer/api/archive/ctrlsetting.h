#ifndef CTRLSETTING_H
#define CTRLSETTING_H
#include "../ctrlapi.h"
#include "../dataapi.h"

class CtrlSetting : public CtrlApi, DataApi
{
public:
    CtrlSetting();
    virtual ~CtrlSetting();
    virtual bool excute(const std::string &cmd) override;
    virtual std::string get(const std::string &param) override;
    virtual bool get(const std::string &param, std::string &data) override;
};

#endif // CTRLSETTING_H
