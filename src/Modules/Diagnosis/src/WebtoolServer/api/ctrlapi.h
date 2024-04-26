#ifndef CTRLAPI_H
#define CTRLAPI_H
#include <string>

class CtrlApi
{
public:
    CtrlApi();
    virtual ~CtrlApi();
    virtual bool excute(const std::string& cmd);
    virtual std::string get(const std::string &param);
    virtual bool get(const std::string &param, std::string &data);
};

#endif // CTRLAPI_H
