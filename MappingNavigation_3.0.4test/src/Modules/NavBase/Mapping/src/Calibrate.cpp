

#include "Calibrate.h"

#include "calibration.h"

#include <stdio.h>
#include <fstream>

#include "json/json.h"
#include "RoboLocProto.h"
#include "RoboLocClnt.h"
#include "ParameterObject.h"
#include "blackboxhelper.hpp"
#include "LCMTask.h"

#include "Tools.h"
#include "RawMap.h"
#include "BaseOdometry.h"
#include "SensorFamily.h"




#define MAPPING_CTRL_CYCLE      20

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

//////////////////////////////////////////////////////////////////////////////
//  calibrate laser by odom
//   Author: lishen
//   Date:   2023. 2.



//
// by lishen
//

namespace mapping
{
//   The support routine of record Dx
void* RecordingDxSupportProc(LPVOID pParam)
{
    CCalibrate* pCalibrate = reinterpret_cast<CCalibrate*>(pParam);
    while (WaitForSingleObject(pCalibrate->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
       // pCalibrate->m_status = Calibrate_RECORDING_DX;
        pCalibrate->SupportRoutine();
        Sleep(MAPPING_CTRL_CYCLE);
    }


    double lasertf[12];
    int result = 10;

    for(int i= 0;i<12;i++)
        lasertf[i]=0;

    if(pCalibrate->SaveFile())
    {
        result = CalibrateLaser(WORK_PATH"CalibrateDataSet.dx", lasertf, 0);
        lasertf[6] = lasertf[3];
        lasertf[7] = lasertf[4];
        lasertf[8] = lasertf[5];
    }

    //double initlasertf[6];
    auto pAFamily = SensorFamilySingleton::GetInstance();
    sensor::CSensorData* sensor_data = NULL;

    for ( unsigned int m = 0; m < pAFamily->GetCount(); m++ ) {
        sensor_data = pAFamily->GetSensorData(m);
        if ( sensor_data && sensor_data->parm && m<2) {
            lasertf[(2*m+1)*3] = sensor_data->parm->m_pst.x;
            lasertf[(2*m+1)*3+1] = sensor_data->parm->m_pst.y;
            lasertf[(2*m+1)*3+2] = sensor_data->parm->m_pst.fThita*180.0/PI;
        }
    }

    LCMTask::GetLcmInstance().SendCalibrationResult(result, lasertf);

    SetEvent(pCalibrate->m_hThreadDead);
    pthread_exit(NULL);

    return NULL;
}
/*
void* RecordingDxSupportProc(LPVOID pParam)
{
    CCalibrate* pCalibrate = reinterpret_cast<CCalibrate*>(pParam);
    while (WaitForSingleObject(pCalibrate->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
       // pCalibrate->m_status = Calibrate_RECORDING_DX;
        pCalibrate->SupportRoutine();
        Sleep(MAPPING_CTRL_CYCLE);
    }


    double lasertf[12];
    int result = 10;

    if(pCalibrate->SaveFile())
    {

        result = CalibrateLaser("ReflectorPointssick.dx", lasertf, 0);

        //if(result == CLBT_SUCCESS_ONE_LASER || result == CLBT_SUCCESS_TWO_LASER)
        //    pCalibrate->WriteLaserParam(result, lasertf);
    }

    //double initlasertf[6];
    auto pAFamily = SensorFamilySingleton::GetInstance();
    sensor::CSensorData* sensor_data = NULL;

    for ( unsigned int m = 0; m < pAFamily->GetCount(); m++ ) {
        sensor_data = pAFamily->GetSensorData(m);
        if ( sensor_data && sensor_data->parm && m<2) {
            lasertf[6+m*3] = sensor_data->parm->m_pst.x;
            lasertf[6+m*3+1] = sensor_data->parm->m_pst.y;
            lasertf[6+m*3+2] = sensor_data->parm->m_pst.fThita*180.0/PI;
        }
    }

    LCMTask::GetLcmInstance().SendCalibrationResult(result, lasertf);

    SetEvent(pCalibrate->m_hThreadDead);
    pthread_exit(NULL);

    return NULL;
}
*/
CCalibrate::CCalibrate()
{

    m_dResolution = 0.05;


}

CCalibrate::~CCalibrate()
{
    StopRecordDx();
}



bool CCalibrate::StartRecordDx()
{
    if (m_bStarted) {
        return true;
    }

    m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);

    m_dResolution = 0.05;

    // clear the odometry
    auto pOdometry = BaseOdomSingleton::GetInstance();
    CPosture odom_trans;
    pOdometry->SetAccumuOdom(0);    //单位为毫米
    //pOdometry->GetLocalOdomTrans(odom_trans);

    // get the mapping dataset size
    int dataset_size = RAW_MAP_CAPACITY_MAPPING;
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Mapping_MappingDataSetSize", dataset_size);

    // clear the raw map space
    auto pRawMap = RawMapSingleton::GetInstance();
    pRawMap->Clear();
    pRawMap->SetMaxCount(dataset_size);
    pRawMap->SetStartTime(m_nStartTime);


    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return false;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return false;

    // Start the support procedure
    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_pMappingThread, &attr, RecordingDxSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RecordingDxSupportProc Pthread Failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Creat RecordingDxSupportProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr);

    m_bFirstTime = true;
    m_bStarted = true;

    return true;
}

bool CCalibrate::StopRecordDx()
{
    if (!m_bStarted)
        return false;

    std::cout<<"StopRecordDx"<<std::endl;

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 50000000000000000);  //???

    PthreadJoin(m_pMappingThread);

    if (m_hKillThread != NULL)
    {
        CloseHandle(m_hKillThread);
        m_hKillThread = NULL;
    }

    if (m_hThreadDead != NULL)
    {
        CloseHandle(m_hThreadDead);
        m_hThreadDead = NULL;
    }

    m_pMappingThread = 0;
    m_nStartTime = 0;
    m_bStarted = false;

    return true;
}



bool CCalibrate::WriteLaserParam(int res,double *lasertf)
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm)
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        int SensorCount = 0;
        if (!LaserParmRoot["laser"].isNull())
        {
            SensorCount = LaserParmRoot["laser"].size();
        }

        for(int i = 0; i < SensorCount; i++)
        {

           /* if (!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["StartAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["StartAngle"] = str;

            }

            if (!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["EndAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["EndAngle"] = str;
            }*/

            if(i==0 ||(res == CLBT_SUCCESS_TWO_LASER&&i==1))
            {

                int kk = 0;
                if(i==0)
                    kk = 0;
                if(i==1)
                    kk = 6;

                if (!LaserParmRoot["laser"][i]["x"].isNull()) {
                    LaserParmRoot["laser"][i]["x"] = lasertf[kk+0];
                }


                if (!LaserParmRoot["laser"][i]["y"].isNull()) {
                    LaserParmRoot["laser"][i]["y"] = lasertf[kk+1];
                }

                if (!LaserParmRoot["laser"][i]["thita"].isNull()) {
                    LaserParmRoot["laser"][i]["thita"] = lasertf[kk+2];
                }
            }

           /* if (!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MaxRange"] = str;
            }

            if (!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MinRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MinRange"] = str;
            }

            int nRangeCount = 0;
            if (!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                nRangeCount = LaserParmRoot["laser"][i]["VisualRange"].size();
            }

            for(int j = 0; j < nRangeCount; j++)
            {
                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"] = str;
                }

                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"] = str;
                }
            }*/
        }
    }
    FileLaserParm.close();


    std::ofstream wFileLaserParm(WORK_PATH"LaserParm1.json",std::ios::out );
    Json::StyledWriter WJ;
    if(!wFileLaserParm.is_open())
    {
         std::cout<<"open write faid\n";

    }
    else
    {
         wFileLaserParm<<WJ.write(LaserParmRoot);
    }
    wFileLaserParm.close();


    return bRet;
}



bool CCalibrate::StartCalibration()
{

    if (m_bStarted) {
        return true;
    }

    bool res = StartRecordDx();
    return res;

}

bool CCalibrate::StopCalibration()
{
    if (!m_bStarted)
        return false;

    if(!StopRecordDx()) {
        return false;
    }

    return true;
}


}



