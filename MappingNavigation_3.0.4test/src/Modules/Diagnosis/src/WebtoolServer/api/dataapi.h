#ifndef DATAAPI_H
#define DATAAPI_H
#include <string>

class DataApi
{
public:
    DataApi();
    virtual ~DataApi();
    virtual std::string get(const std::string& param);
    virtual bool get(const std::string& param, std::string & data);
};

#endif // DATAAPI_H
