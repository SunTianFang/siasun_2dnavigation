#include <iostream>
#include <unistd.h>
#include <thread>
#include "RoboLocClnt.h"
#include "AvoidObstacle.h"
#include "Project.h"
#include "PLS.h"
#include "blackboxhelper.hpp"

#ifdef USE_BLACK_BOX
extern CBlackBox LaserBox;
#endif


namespace pls {

CPlsManager::CPlsManager()
{

}

CPlsManager::~CPlsManager()
{

}

void CPlsManager::Clear()
{

}

bool CPlsManager::Initialize()
{
    if(GetParamByJson())
    {
         printf("CPlsManager::Initialize !!!!!!!! GetParamByJson true\n");
        std::map<std::string,CAvoidObstacle>::iterator it;
        for(it = m_LaserMap.begin(); it != m_LaserMap.end();it++)
        {
            m_LayerSetting[it->first] = 1;
            m_ObstacleResult[it->first] = 3;
        }
        return true;
    }
    else
    {
         printf("CPlsManager::Initialize !!!!!!!! GetParamByJson false\n");
#ifdef USE_BLACK_BOX
          FILE_BlackBox(LaserBox, "CPlsManager::Initialize !!!!!!!! GetParamByJson false");
#endif
        return false;
    }
}

void CPlsManager::SupportRoutine()
{
    printf("CPlsManager::SupportRoutine!!!\n");
    Initialize();
    UpdateData();
    while(1)
    {
        auto pRoboClnt = RoboClntSingleton::GetInstance();
        map<string,int>::iterator it;
        //printf("m_LayerSetting size is %d\n",m_LayerSetting.size());
        for(it = m_LayerSetting.begin();it != m_LayerSetting.end();it++)
        {
            unsigned int key;
            int layer = pRoboClnt->GetPlsLayer(it->first,key);
            if(layer != -1)
            {
                SetLaserLayer(it->first,layer,key);
            }
        }
        SendData();
        usleep(16000);
    }
    //printf("CPlsManager::SupportRoutine end!!!\n");
}

bool CPlsManager::Stop()
{

}

void PlsThread(std::string ip)
{
    PlsManagerSingleton::GetInstance()->m_LaserMap[ip].SetLaserIP(ip);
    PlsManagerSingleton::GetInstance()->m_LaserMap[ip].SupportRoutine();
}

void CPlsManager::UpdateData()
{
    std::map<std::string,CAvoidObstacle>::iterator it;
    for(it = m_LaserMap.begin(); it != m_LaserMap.end();it++)
    {
        std::thread AvoidThrd(PlsThread,it->first);
        AvoidThrd.detach();
    }
}

void CPlsManager::SendData()
{
    //printf("CPlsManager::SendData() m_LaserMap size is %d\n",m_LaserMap.size());
    std::map<std::string,CAvoidObstacle>::iterator it;
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    for(it = m_LaserMap.begin(); it != m_LaserMap.end();it++)
    {
        m_ObstacleResult[it->first] = it->second.GetSpeedType();
      //  printf("CPlsManager::SendData() %s, %d,m_LayerSetting %d\n",it->first.c_str(),m_ObstacleResult[it->first],m_LayerSetting[it->first]);
        pRoboClnt->UpdatePlsState(it->first,m_LayerSetting[it->first],m_ObstacleResult[it->first],it->second.GetWorkState(),it->second.GetKey());
    }
}

bool CPlsManager::GetParamByJson()
{
//    std::ifstream programFile(WORK_PATH"PlsProgramParam.json");
//    int time,count;
//    if(programFile)
//    {
//        Json::Reader programRead;
//        Json::Value programRoot;
//        programRead.parse(programFile, programRoot, false);
//        time = programRoot["time"].asInt();
//        count = programRoot["count"].asInt();
//    }
//    else
//    {
//        printf("CPlsManager::GetParamByJson PlsProgramParam read failed!!!!!\n");
//    }
    std::ifstream initFile(WORK_PATH"SoftPlsParam.json");
    if (!initFile)
    {
        std::cout<<"  GetParamByJson  can not open file  SoftPlsParam.json "<<std::endl;

#ifdef USE_BLACK_BOX
         FILE_BlackBox(LaserBox, "GetParamByJson  can not open file path :",WORK_PATH,"SoftPlsParam.json");
#endif
        return false;
    }
    Json::Reader read;
    Json::Value root;
    read.parse(initFile, root, false);
    for (int d = 0; d < root.size();d++)
    {
        CAvoidObstacle avoid;
        std::string IP = root[d]["IP"].asString();


        for (int i = 0; i < root[d]["pls"].size(); i++)
        {
            int num = root[d]["pls"][i]["index"].asInt();
            Area_t tempArea;
            if (!root[d]["pls"][i]["warning"].isNull())
            {
                for (int j = 0; j < root[d]["pls"][i]["warning"].size(); j++)
                {
                    Point_t tmp;
                    tmp.x = root[d]["pls"][i]["warning"][j]["x"].asInt();
                    tmp.y = root[d]["pls"][i]["warning"][j]["y"].asInt();
                    tempArea.WarningPointVector.push_back(tmp);
                }
            }
            if (!root[d]["pls"][i]["center"].isNull())
            {
                for (int j = 0; j < root[d]["pls"][i]["center"].size(); j++)
                {
                    Point_t tmp;
                    tmp.x = root[d]["pls"][i]["center"][j]["x"].asInt();
                    tmp.y = root[d]["pls"][i]["center"][j]["y"].asInt();
                    tempArea.CenterPointVector.push_back(tmp);
                }
            }
            if (!root[d]["pls"][i]["stop"].isNull())
            {
                for (int j = 0; j < root[d]["pls"][i]["stop"].size(); j++)
                {
                    Point_t tmp;
                    tmp.x = root[d]["pls"][i]["stop"][j]["x"].asInt();
                    tmp.y = root[d]["pls"][i]["stop"][j]["y"].asInt();
                    tempArea.StopPointVector.push_back(tmp);
                }
            }
            AvoidParam tempParam;
            tempParam.centerDetectionPeriod = 1;
            tempParam.stopDetectionPeriod = 1;
            tempParam.warningDetectionPeriod = 1;
            tempParam.centerPointNum = 15;
            tempParam.stopPointNum = 15;
            tempParam.warningPointNum = 15;
            if (!root[d]["pls"][i]["param"].isNull())
            {
                 tempParam.centerDetectionPeriod = root[d]["pls"][i]["param"]["centerperiod"].asInt();
                 tempParam.stopDetectionPeriod = root[d]["pls"][i]["param"]["stopperiod"].asInt();               
                 tempParam.warningDetectionPeriod = root[d]["pls"][i]["param"]["warningperiod"].asInt();
                 tempParam.centerPointNum = root[d]["pls"][i]["param"]["centerpointnum"].asInt();
                 tempParam.stopPointNum = root[d]["pls"][i]["param"]["stoppointnum"].asInt();
                 tempParam.warningPointNum = root[d]["pls"][i]["param"]["warningpointnum"].asInt();
            }
            avoid.m_AreaMap[num] = tempArea;
            avoid.m_param[num] = tempParam;
        }
//        avoid.UpdatePlsParam(time,count);
        m_LaserMap[IP] = avoid;
    }
    return true;
}

void CPlsManager::SetLaserLayer(std::string ip,int layer,unsigned int key)
{
   // printf("CPlsManager::SetLaserLayer ip %s,layer %d\n",ip.c_str(),layer);
    m_LaserMap[ip].SetKey(key);
    if(m_LaserMap[ip].SetAreaLayer(layer))
    {
        m_LayerSetting[ip] = layer;
    }
}

}
