#pragma once


#include <vector>
#include "json/json.h"
#include <fstream>

#define PAI 3.1415926
#define Max_Dis 30000



typedef struct{
    int x;
    int y;
}Point_t;

typedef struct{
    std::vector<Point_t> StopPointVector;
    std::vector<Point_t> CenterPointVector;
    std::vector<Point_t> WarningPointVector;
}Area_t;

typedef struct{
    int stopPointNum;
    int centerPointNum;
    int warningPointNum;
    int stopDetectionPeriod;
    int centerDetectionPeriod;
    int warningDetectionPeriod;
}AvoidParam;

class CAvoidObstacle
{
    public:
        ~CAvoidObstacle();
        CAvoidObstacle();
        bool UpdateData(const std::vector<int> &data,const std::vector< int > &data_intensity,int len );
        void UpdateGetSpeedType( void );
        bool IsPointOnLine(int pointX,int pointY,int lineXA,int lineYA,int lineXB,int lineYB);
        bool IsIntersect(int lineXA,int lineYA,int lineXB,int lineYB,int lineXC,int lineYC,int lineXD,int lineYD);
        bool PointInPolygon(int pointX,int pointY,std::vector<Point_t> POL);
        void SupportRoutine();
        void SetLaserIP(std::string ip)
        {
            m_sLaserIP = ip;
        }
        bool SetAreaLayer(int num)
        {
           // printf("CAvoidObstacle SetAreaLayer num is %d\n",num);
            if(num == 15)
            {
                //m_bIsEnable = false;
                //true;
            }
            else
            {
                m_bIsEnable = true;
            }
            if(num < 0 || m_AreaMap.count(num) == 0)
            {
                //m_iAreaIndex=1;
                if(m_iWorkState == 1)
                    m_iWorkState = 3;
                return false;
            }
            if(m_iWorkState== 3)
                m_iWorkState = 1;
            m_iAreaIndex = num;
            return true;
        }
        void SetKey(unsigned int key)
        {
            m_iKey = key;
        }
        unsigned int GetKey()
        {
            return m_iKey;
        }
        int GetSpeedType()
        {
            return m_iSpeedType; //0:停止区 1:减速区 2:警告区
        }
        void UpdatePlsParam(int time,int count)
        {
            printf("UpdatePlsParam time %d,count %d\n",time,count);
            m_iUpdateTime = time;
            m_iLaserErrorCount = count;
        }
        int GetWorkState()
        {
            return m_iWorkState;
        }
    public:
        std::map<int,Area_t> m_AreaMap;
        std::map<int,AvoidParam> m_param;
    private:
        int m_iSpeedType;
        int m_iAreaIndex;
        bool m_bIsEnable;
        double m_dPreTheta; //激光每根线的间隔角度
        double m_dStartTheta; //激光第一根线的角度
        std::string m_sLaserIP;
        pthread_mutex_t m_LaserData_mutex;
        std::vector< int > m_LaserData;
        int m_iLaserErrorFlag;
        int m_iUpdateTime;
        int m_iLaserErrorCount;
        int m_iStopNum;
        int m_iCenterNum;
        int m_iWarningNum;
        int m_iWorkState;
        unsigned int m_iKey;

};

