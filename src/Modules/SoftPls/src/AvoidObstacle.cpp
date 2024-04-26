#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>

#include "LinuxSetting.h"
#include "AvoidObstacle.h"
#include "SensorFamily.h"
#include "blackboxhelper.hpp"

#ifdef USE_BLACK_BOX
extern CBlackBox LaserBox;
#endif

using namespace std;

CAvoidObstacle::CAvoidObstacle()
{
    m_iAreaIndex = 1;
    m_bIsEnable = true;
    m_iSpeedType = 3;
    m_iLaserErrorFlag = 0;
    m_iUpdateTime = 50000;
    m_iLaserErrorCount = 3;
    m_iStopNum = 0;
    m_iCenterNum = 0;
    m_iWarningNum = 0;
    m_iWorkState = 1;
}
CAvoidObstacle::~CAvoidObstacle()
{
    pthread_mutex_destroy(&m_LaserData_mutex);
}


bool CAvoidObstacle::UpdateData(
                        const vector< int > &data,
						const std::vector< int > &data_intensity,
                        int len )
{
   m_LaserData.clear();
   for( int i = 0; i < len; i++)
   {
      m_LaserData.push_back(data.at(i));
   }
   UpdateGetSpeedType();
   //pthread_mutex_unlock(&m_LaserData_mutex);
   return true;
}

void CAvoidObstacle::SupportRoutine()
{
    auto pFamily = SensorFamilySingleton::GetInstance();
    std::cout<<m_sLaserIP<<"CAvoidObstacle::SupportRoutine begin"<<std::endl;
    if(m_AreaMap.size() == 0)
    {
        printf("IP %s ,Do not have any area data!!!!!!!!!!!!!!!!!!\n",m_sLaserIP.c_str());
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LaserBox, "CAvoidObstacle::SupportRoutine Do not have any area data");
#endif
        return;
    }
    while(1)
    {
        for(int i = 0;i < pFamily->GetCount();i++)
        {
            if(pFamily->GetState(i) && pFamily->DataReady(i))
            {
                auto scan = pFamily->GetSensorData(i);
                //printf("CAvoidObstacle::SupportRoutine scan->parm->strIP %s,m_sLaserIP %s\n",scan->parm->strIP.c_str(),m_sLaserIP.c_str());
                if(scan->parm->strIP == m_sLaserIP)
                {
                    //printf("pFamily->GetCount %d,time is %lld\n",pFamily->GetCount(),GetTickCount());
                    m_dStartTheta = scan->parm->m_fStartAngle;
                    m_dPreTheta = fabs((scan->parm->m_fStartAngle - scan->parm->m_fEndAngle)/scan->parm->m_nLineCount);
                    //std::cout<<"m_dStartTheta is "<<m_dStartTheta<<",m_dPreTheta is "<<m_dPreTheta<<",size is "<<scan->scanner->m_nPointCount<<",scan->parm->m_fStartAngle is "<<scan->parm->m_fStartAngle<<std::endl;
                    int size = scan->scanner->m_nPointCount;
                    vector<int>  data;
                    vector<int> intensity;
                    std::shared_ptr<sensor::CRawPointCloud> pCloud;
                    scan->scanner->GetRawPointCloud(pCloud);
                    //printf("pCloud sync time %lld,raw time %lld\n",pCloud->timestamp_sync,pCloud->timestamp_raw);

                    float angular_increment = 0.0;
                    if(size > 0) {
                        angular_increment = ( scan->parm->m_fEndAngle - scan->parm->m_fStartAngle) / size;
                    }

                    for(int j = 0;j < size;j++)
                    {

                        float a = m_dStartTheta + j*angular_increment;

                        int dis = pCloud->distance[j];

                        // 过虑掉不满足可视角度的点
                        if(!scan->parm->m_AppAngleRange.Contain(a)){
                            dis = 0.0;
                        }


                        //printf("dis is %d,j is %d\n",dis,j);
                        data.push_back(dis);
                        intensity.push_back(500);
                    }
                    if(pFamily->IsBlocked())//500ms
                    {
                        printf("IP %s SupportRoutine IsBlocked !!!!!\n",m_sLaserIP.c_str());
#ifdef USE_BLACK_BOX
                        FILE_BlackBox(LaserBox, "CAvoidObstacle::SupportRoutine pFamily->IsBlocked()");
#endif
                        m_iWorkState = 2;
                    }
                    else
                    {
                        if(m_iWorkState == 2)
                             m_iWorkState = 1;
                        UpdateData(data,intensity,size);
                    }
                }
            }
        }
        usleep(m_iUpdateTime);
    }
    //std::cout<<m_sLaserIP<<"CAvoidObstacle::SupportRoutine end"<<std::endl;
}

void CAvoidObstacle::UpdateGetSpeedType( void )
{
    int size = m_LaserData.size();
    int StopCnt = 0;
    int CenterCnt = 0;
    int WarningCnt = 0;
    for(int i=0;i<size;i++)
    {
        Point_t laser;
        // 11.11 suxiang
        // 10.18 dq 2m点云
        //if(m_LaserData[i] == 0 || m_LaserData[i] > 50000)
        if(m_LaserData[i] < 80 || m_LaserData[i] > 50000)
        {
            continue;
        }
        laser.x = m_LaserData[i]*cos(m_dStartTheta + m_dPreTheta*i);
        laser.y = m_LaserData[i]*sin(m_dStartTheta + m_dPreTheta*i);
        //printf("UpdateGetSpeedType m_dStartTheta is %f,m_dPreTheta is %f,cos %f,sin %f,m_LaserData[i] %d,i %d,x %d,y %d\n",m_dStartTheta,m_dPreTheta,cos(m_dStartTheta + m_dPreTheta*i),sin(m_dStartTheta + m_dPreTheta*i),m_LaserData[i],i,laser.x,laser.y);
        Area_t tempArea = m_AreaMap[m_iAreaIndex];
        if(PointInPolygon(laser.x,laser.y,tempArea.WarningPointVector))
        {
            WarningCnt++;
        }
        if(PointInPolygon(laser.x,laser.y,tempArea.CenterPointVector))
        {
           // printf("laser.x %d,laser.y %d i %d,m_LaserData[i] %d\n",laser.x,laser.y,i,m_LaserData[i]);
            CenterCnt++;
        }
        if(PointInPolygon(laser.x,laser.y,tempArea.StopPointVector))
        {
            //printf("laser.x %d,laser.y %d i %d,m_LaserData[i] %d\n",laser.x,laser.y,i,m_LaserData[i]);
            StopCnt++;
        }
    }
    unsigned long long timeNow = GetTickCount();
  // printf("StopCnt %d,CenterCnt %d,WarningCnt %d,timeNow %lld,size %d,ip %s\n",StopCnt,CenterCnt,WarningCnt,timeNow,size,m_sLaserIP.c_str());
#ifdef USE_BLACK_BOX
    //FILE_BlackBox(LaserBox, "CAvoidObstacle::UpdateGetSpeedType",",layer is ",m_iAreaIndex,",StopCnt ",StopCnt,",CenterCnt",CenterCnt,",WarningCnt",WarningCnt);
#endif
  //  printf("m_param size is %d,m_iAreaIndex %d,ip %s\n",m_param.size(),m_iAreaIndex,m_sLaserIP.c_str());
    if(StopCnt >= m_param[m_iAreaIndex].stopPointNum)
    {
        if(m_iStopNum < m_param[m_iAreaIndex].stopDetectionPeriod)
            m_iStopNum++;
    }
    else
    {
        m_iStopNum = 0;
    }
    if(CenterCnt >= m_param[m_iAreaIndex].centerPointNum)
    {
        if(m_iCenterNum < m_param[m_iAreaIndex].centerDetectionPeriod)
            m_iCenterNum++;
    }
    else
    {
        m_iCenterNum = 0;
    }
    if(WarningCnt >= m_param[m_iAreaIndex].warningPointNum)
    {
        if(m_iWarningNum < m_param[m_iAreaIndex].warningDetectionPeriod)
            m_iWarningNum++;
    }
    else
    {
        m_iWarningNum = 0;
    }
    if(m_iStopNum >= m_param[m_iAreaIndex].stopDetectionPeriod)
    {
        m_iSpeedType = 0;
    }
    else if(m_iCenterNum >= m_param[m_iAreaIndex].centerDetectionPeriod)
    {
        m_iSpeedType = 1;
    }
    else if(m_iWarningNum >= m_param[m_iAreaIndex].warningDetectionPeriod)
    {
        m_iSpeedType = 2;
    }
    else
    {
        m_iSpeedType = 3;
    }
    if(!m_bIsEnable)
    {
        printf("m_bIsEnable == false\n");
        m_iSpeedType = 3;
    }
    //printf("!!!!!!!!!!!!!!!!!!!!end\n");
    return;
}

bool CAvoidObstacle::IsPointOnLine(int pointX,int pointY,int lineXA,int lineYA,int lineXB,int lineYB)
{
    bool flag = false;
    int d1 = (lineXA - pointX) * (lineYB - pointY) - (lineXB - pointX) * (lineYA - pointY);
    if ((abs(d1) < 1) && ((pointX - lineXA) * (pointX - lineXB) <= 0) && ((pointY - lineYA) * (pointY - lineYB) <= 0))
    {
        flag = true;
    }
    return flag;
}

bool CAvoidObstacle::IsIntersect(int lineXA,int lineYA,int lineXB,int lineYB,int lineXC,int lineYC,int lineXD,int lineYD)
{
    bool flag = false;
    long a = (lineXD-lineXC)*(lineYA-lineYC)-(lineXA-lineXC)*(lineYD-lineYC);
    long b = (lineXD-lineXC)*(lineYB-lineYC)-(lineXB-lineXC)*(lineYD-lineYC);
    long c = (lineXB-lineXA)*(lineYC-lineYA)-(lineXC-lineXA)*(lineYB-lineYA);
    long d = (lineXB-lineXA)*(lineYD-lineYA)-(lineXD-lineXA)*(lineYB-lineYA);
   // printf("IsIntersect a %ld,b %ld,c %ld,d %ld\n",a,b,c,d);
    if(a*b<=0 && c*d<=0)
    {
       flag = true;
    }
    return flag;
}

bool CAvoidObstacle::PointInPolygon(int pointX,int pointY,std::vector<Point_t> POL)
{
    bool isInside = false;
    int count = 0;

    if(POL.size() < 3)
    {
        return isInside;
    }

    //
    int minX = 9999;
    for (int i = 0; i < POL.size(); i++)
    {
        minX = std::min(minX, POL[i].x);
    }

    //
    int px = pointX;
    int py = pointY;
    int linePoint1x = pointX;
    int linePoint1y = pointY;
    int linePoint2x = minX -100;			//取最小的X值还小的值作为射线的终点
    int linePoint2y = pointY;
    //printf("linePoint1x %d,linePoint1y %d,linePoint2x %d,linePoint2y %d,minX %d\n",linePoint1x,linePoint1y,linePoint2x,linePoint2y,minX);
    //遍历每一条边
    for (int i = 0; i < POL.size(); i++)
    {
        int cx1,cy1,cx2,cy2;
        if(i == POL.size() - 1)
        {
            cx1 = POL[i].x;
            cy1 = POL[i].y;
            cx2 = POL[0].x;
            cy2 = POL[0].y;
        }
        else
        {
            cx1 = POL[i].x;
            cy1 = POL[i].y;
            cx2 = POL[i + 1].x;
            cy2 = POL[i + 1].y;
        }

        if (IsPointOnLine(px, py, cx1, cy1, cx2, cy2))
        {
            //printf("IsPointOnLine!!!\n");
            return true;
        }
        if (fabs(cy2 - cy1) < 1)   //平行则不相交
        {
            continue;
        }
        if (IsPointOnLine(cx1, cy1, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy1 > cy2)			//只保证上端点+1
            {
                //printf("111 cx1 %d, cy1 %d,cx2 %d,cy2 %d\n",cx1,cy1,cx2,cy2);
                count++;
            }
        }
        else if (IsPointOnLine(cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy2 > cy1)			//只保证上端点+1
            {
                //printf("222 cx1 %d, cy1 %d,cx2 %d,cy2 %d\n",cx1,cy1,cx2,cy2);
                count++;
            }
        }
        else if (IsIntersect(cx1, cy1, cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))   //已经排除平行的情况
        {
            //printf("333 px %d,py %d,cx1 %d, cy1 %d,cx2 %d,cy2 %d,i %d\n",px,py,cx1,cy1,cx2,cy2,i);
            count++;
        }
    }

    if (count % 2 == 1)
    {
        //printf("count %d !!!\n",count);
        isInside = true;
    }

    return isInside;
}
