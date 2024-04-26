#include "apimanager.h"
#include "archive/datasetting.h"
#include "archive/ctrlsetting.h"
#include "archive/datalocation.h"
#include "archive/datapcloud.h"
#include "archive/dataparm.h"
#include "archive/datadxlaserparm.h"
#include "archive/datadxscandata.h"
#include "archive/datastartrecorddx.h"
#include "archive/datafinishrecorddx.h"
#include "../server/httpServer.h"
#include "json/json.h"
#include <fstream>

static bool checkPost(const std::string &param)
{
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(param, root, false))
    {
        return false;
    }
    auto group = root["group"].asString();
    auto account = root["account"].asString();
    auto password = root["password"].asString();
    if (group == "siasun" && account == "test" && password == "123456")
    {
        return true;
    }
    else if (group == "siasun" && account == "test" && password == "123456")
    {
        return true;
    }
    else
    {
        return false;
    }
}

ApiManager::ApiManager()
{
}

void ApiManager::config()
{
    m_dataApi["laserParm"] = std::make_shared<DataParm>();
    m_dataApi["pointCloud"] = std::make_shared<DataPCloud>();
    m_dataApi["location"] = std::make_shared<DataLocation>();
    m_dataApi["setting"] = std::make_shared<DataSetting>();
    m_dataApi["writeDxLaserParm"] = std::make_shared<DataDxLaserParm>();
    m_dataApi["writeDxScanData"] = std::make_shared<DataDxScanData>();
    m_ctrlApi["setting"] = std::make_shared<CtrlSetting>();
    m_dataApi["startRecordDx"] = std::make_shared<DataStartRecordDx>();
    m_dataApi["finishRecordDx"] = std::make_shared<DataFinishRecordDx>();

   
    HTTP_SERVER_OBJ.addPost(
        "/api/data/laserParm", [&](const std::string &msg, std::string &resp) {
            bool ok = get("laserParm", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/pointCloud", [&](const std::string &msg, std::string &resp) {
            bool ok = get("pointCloud", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/location", [&](const std::string &msg, std::string &resp) {
            bool ok = get("location", msg, resp);
            return ok ? 200 : 400;
        });
    //get
    HTTP_SERVER_OBJ.addPost(
        "/api/data/setting", [&](const std::string &msg, std::string &resp) {
            bool ok = get("setting", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/writeDxLaserParm", [&](const std::string &msg, std::string &resp) {
            bool ok = get("writeDxLaserParm", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/writeDxScanData", [&](const std::string &msg, std::string &resp) {
            bool ok = get("writeDxScanData", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/startRecordDx", [&](const std::string &msg, std::string &resp) {
            bool ok = get("startRecordDx", msg, resp);
            return ok ? 200 : 400;
        });
    HTTP_SERVER_OBJ.addPost(
        "/api/data/finishRecordDx", [&](const std::string &msg, std::string &resp) {
            bool ok = get("finishRecordDx", msg, resp);
            return ok ? 200 : 400;
        });
    //set
    HTTP_SERVER_OBJ.addPost(
        "/api/ctrl/setting", [&](const std::string &msg, std::string &resp) {
            bool ok = set("setting", msg);
            bool ok_get = okGet("setting", msg, resp);
//            resp = "ok";
            return ok ? 200 : 400;
        });
}

bool ApiManager::okGet(const std::string &api, const std::string &param, std::string &data)
{
    if (m_ctrlApi.find(api) == m_ctrlApi.end())
    {
        return false;
    }
    if (!m_ctrlApi[api])
    {
        return false;
    }
    return m_ctrlApi[api]->get(param, data);
}

bool ApiManager::get(const std::string &api, const std::string &param, std::string &data)
{
    if (m_dataApi.find(api) == m_dataApi.end())
    {
        return false;
    }
    if (!m_dataApi[api])
    {
        return false;
    }
    return m_dataApi[api]->get(param, data);
}

bool ApiManager::set(const std::string &api, const std::string &param)
{
    if (m_ctrlApi.find(api) == m_ctrlApi.end())
    {
        return false;
    }
    if (!m_ctrlApi[api])
    {
        return false;
    }
    return m_ctrlApi[api]->excute(param);
}

bool ApiManager::download(const std::string &api, const std::string &msg, std::string &path, std::string &file)
{
    if (m_downloadApi.find(api) == m_downloadApi.end())
    {
        return false;
    }
    if (!m_downloadApi[api])
    {
        return false;
    }
    return m_downloadApi[api]->handle(msg, path, file);
}
