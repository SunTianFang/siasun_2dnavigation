#ifndef APIMANAGER_H
#define APIMANAGER_H
#include "dataapi.h"
#include "ctrlapi.h"
#include "downloadapi.h"
#include <memory>
#include <map>
#include <string>

class ApiManager
{
public:
    ApiManager();
    void config();    
    bool okGet(const std::string &api, const std::string &param, std::string &data);
    bool get(const std::string& api, const std::string& param, std::string& data);
    bool set(const std::string& api, const std::string& param);
    bool download(const std::string& api, const std::string &msg, std::string &path, std::string &file);
private:
    std::map<std::string,std::shared_ptr<DataApi>> m_dataApi;
    std::map<std::string,std::shared_ptr<CtrlApi>> m_ctrlApi;
    std::map<std::string,std::shared_ptr<DownloadApi>> m_downloadApi;
};

#endif // APIMANAGER_H
