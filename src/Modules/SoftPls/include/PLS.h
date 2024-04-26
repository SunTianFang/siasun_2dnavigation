#pragma once

#include <stdio.h>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include "MagicSingleton.h"

class CAvoidObstacle;
namespace pls {

class CPlsManager
{
private:
    CPlsManager();
    ~CPlsManager();

    friend MagicSingleton<CPlsManager>;
public:
    bool Initialize();

    void Clear();

    void SupportRoutine();

    bool Stop();

    void SetLaserLayer(std::string ip,int layer,unsigned int key);
private:
    void UpdateData();
    void SendData();
    bool GetParamByJson();
private:
    std::map<std::string,int> m_LayerSetting;
    std::map<std::string,int> m_ObstacleResult;
public:
    std::map<std::string,CAvoidObstacle> m_LaserMap;
};

} // namespace pls

using PlsManagerSingleton = MagicSingleton<pls::CPlsManager>;




