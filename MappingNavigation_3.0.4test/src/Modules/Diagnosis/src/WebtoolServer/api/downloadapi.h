#ifndef DOWNLOADAPI_H
#define DOWNLOADAPI_H
#include <string>

class DownloadApi
{
public:
    DownloadApi();
    virtual ~DownloadApi();
    virtual bool handle(const std::string &msg, std::string& path, std::string& file);
};

#endif // DOWNLOADAPI_H
