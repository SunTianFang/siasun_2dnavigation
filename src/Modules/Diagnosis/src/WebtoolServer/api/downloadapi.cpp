#include "downloadapi.h"

DownloadApi::DownloadApi()
{

}

DownloadApi::~DownloadApi()
{

}

bool DownloadApi::handle(const std::string &msg, std::string &path, std::string &file)
{
    (void)(msg);
    (void)(path);
    (void)(file);
    return false;
}
