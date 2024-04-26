//
//   The interface of class "CLaserMapping".
//

#include "LaserMapping.h"
#include <stdio.h>
#include <fstream>
#include "Tools.h"
#include "RawMap.h"
#include "BaseOdometry.h"
#include "SensorFamily.h"
#include "json/json.h"
#include "RoboLocProto.h"
#include "RoboLocClnt.h"
#include "ParameterObject.h"
#include "blackboxhelper.hpp"
#include "HttpCommunicationGlobalData.h"


#define MAPPING_CTRL_CYCLE      20

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

//////////////////////////////////////////////////////////////////////////////
//   The support routine of laser mapping.
void* LaserMappingSupportProc(LPVOID pParam)
{
    mapping::CLaserMapping* pMapping = reinterpret_cast<mapping::CLaserMapping*>(pParam);
    while (WaitForSingleObject(pMapping->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
        pMapping->SupportRoutine();
        Sleep(MAPPING_CTRL_CYCLE);
    }

    SetEvent(pMapping->m_hThreadDead);
    pthread_exit(NULL);

    return NULL;
}

namespace mapping {

CLaserMapping::CLaserMapping()
{
    m_hKillThread = NULL;
    m_hThreadDead = NULL;
    m_pMappingThread = 0;
    m_bStarted = false;
    m_nStartTime = 0;
    m_dResolution = 0.2;
    m_bFirstTime = false;
    m_aWorkMode = RLP_MODE_STANDBY;
    m_CurTimeStamp = 0;
    m_PreTimeStamp = 0;
    m_aFileSaving = false;
}

CLaserMapping::~CLaserMapping()
{
    Stop();
}

void CLaserMapping::SupportRoutine()
{
    UpdateOdometry();
    UpdateScan();
}

bool CLaserMapping::Stop()
{
    if (!m_bStarted)
        return false;

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
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
    m_bStarted = false;
    m_nStartTime = 0;

    return true;
}

bool CLaserMapping::UpdateOdometry()
{
    //auto pOdometry = BaseOdomSingleton::GetInstance();
    //pOdometry->UpdateEstimateOdom();
    return true;
}

bool CLaserMapping::UpdateScan()
{
    bool bRet = false;
    CPosture odom_pst;
    unsigned long long timeNow = GetTickCount();
    auto pOdometry = BaseOdomSingleton::GetInstance();
    double acc_odom = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;

    if(!m_bFirstTime && acc_odom < m_dResolution) {
        return false;
    }
    if(m_bFirstTime) {
        bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);
        if(bRet){
            m_PreOdomPst = odom_pst;
        }
        m_PreTimeStamp = timeNow;
        m_bFirstTime = false;
    }
    pOdometry->SetAccumuOdom(0);
    m_CurTimeStamp = timeNow;

    // 获取机器人的相对里程姿态,当前速度矢量,里程计坐标系的全局位姿
    sensor::COdometryData odom_data;
    sensor::CRawScan pRawScan;
    CPosture odom_trans;
    StVelocity base_vel;

    odom_data.odom_flag = pOdometry->GetOdomFlag();
    odom_data.time_stamp = timeNow;
    base_vel = pOdometry->GetBaseVelocity();
    odom_data.velocity.fXLinear = base_vel.Vx / 1000.0;
    odom_data.velocity.fYLinear = base_vel.Vy / 1000.0;
    odom_data.velocity.fAngular = base_vel.Vtheta / 1000.0;

    // 采集建模数据时,根据时间获取位姿,计算两次的位姿变换量,因为车辆中途会长时间停滞,导致按照时间间隔获取位姿变化会出错.
    bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);
    if(bRet){
        m_CurOdomPst = odom_pst;
    }

    CTransform transOdom;
    transOdom.Init(m_PreOdomPst);
    odom_trans = transOdom.GetLocalPosture(m_CurOdomPst);
    // [-PI, PI]
    odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);

    odom_data.local_pst = odom_trans;
    odom_data.global_pst = m_CurOdomPst;

    std::cout << "CLaserMapping transform: x = " << odom_trans.x << ", y = " << odom_trans.y
              << ", thita = " << odom_trans.fThita << std::endl;

#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox, "MapTran:", odom_trans.x, ",", odom_trans.y,
                  ",", odom_trans.fThita, ",", static_cast<int>(timeNow));
#endif

    pRawScan.SetOdometry(odom_data);

    // 获取激光传感器点云数据
    bool bActiveCloud = true;
    auto pAFamily = SensorFamilySingleton::GetInstance();
    std::shared_ptr<sensor::CRawPointCloud> pRawCloud;
    for (unsigned int i = 0; i < pAFamily->GetCount(); i++) {
        if(!pAFamily->GetState(i)|| (pAFamily->GetType(i) == LEIMOUF30)) {
            continue;
        }
        if(!pAFamily->GetRawPointCloud(i, pRawCloud)){
            bActiveCloud = false;
            //continue;
        }
        pRawScan.PushBackPointCloud(pRawCloud);
        //pRawCloud.reset();
        //pRawCloud = nullptr;
    }

    // 建模数据集
    auto pRawMap = RawMapSingleton::GetInstance();
    if(bActiveCloud) {
        pRawMap->AddRawScan(pRawScan);
    }

    // 导航控制器和激光传感器通信中断,上报AGV
    short error_code = RLP_NO_ERROR;
    unsigned char pos_mode = RLP_POSITIONING_STOPPED;
    if(pAFamily->IsBlocked()) {
        error_code = RLP_LASER_TIMEOUT;
    }

    // 发送位姿
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(pRoboClnt){
//
#ifdef USE_LEG_METHOD
        pRoboClnt->SetRoboPose(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, error_code, error_code, pos_mode); // By Sam for LegMethod
#else
          pRoboClnt->SetRoboPose(0, 0, 0, 0, 0, 0, error_code, pos_mode);
#endif
    }

    m_PreOdomPst = m_CurOdomPst;
    m_PreTimeStamp = m_CurTimeStamp.load();
    return true;
}

#if 0
bool CLaserMapping::WriteLaserParm(unsigned int nStartTime)
{
    FILE* fp = NULL;
    bool bRet = false;

    if(m_aWorkMode.load() == RLP_MODE_MAPPING) {
        fp = fopen(WORK_PATH"ReflectorPoints.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(WORK_PATH"ReflectorPoints.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_LOCALIZATION) {
        fp = fopen(LOG_FILE_PATH"ReflectorPointsLog.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(LOG_FILE_PATH"ReflectorPointsLog.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_CALIBRATION) {
        fp = fopen(WORK_PATH"CalibrateDataSet.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(WORK_PATH"CalibrateDataSet.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else {
        return false;
    }

    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteLaserParm(fp);
    fclose(fp);

    return bRet;
}
#endif
bool CLaserMapping::WriteLaserParm(unsigned int nStartTime)
{
    FILE* fp = NULL;
    bool bRet = false;
    bool bPlsLaserWrite = false;

    if(m_aWorkMode.load() == RLP_MODE_MAPPING) {
        fp = fopen(WORK_PATH"ReflectorPoints.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(WORK_PATH"ReflectorPoints.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_LOCALIZATION) {
        fp = fopen(LOG_FILE_PATH"ReflectorPointsLog.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(LOG_FILE_PATH"ReflectorPointsLog.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_CALIBRATION) {
        bPlsLaserWrite = true;
        fp = fopen(WORK_PATH"CalibrateDataSet.dx", "w");
        if(!fp) {
            return false;
        }
        fclose(fp);

        fp = fopen(WORK_PATH"CalibrateDataSet.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else {
        return false;
    }

    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteLaserParm(fp,bPlsLaserWrite);
    fclose(fp);

    return bRet;
}


//
bool CLaserMapping::WriteScanData()
{
    FILE* fp = NULL;
    bool bRet = false;

    if(m_aWorkMode.load() == RLP_MODE_MAPPING) {
        fp = fopen(WORK_PATH"ReflectorPoints.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_LOCALIZATION) {
        fp = fopen(LOG_FILE_PATH"ReflectorPointsLog.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else if(m_aWorkMode.load() == RLP_MODE_CALIBRATION) {
        fp = fopen(WORK_PATH"CalibrateDataSet.dx", "ab");
        if(!fp) {
            return false;
        }
    }
    else {
        return false;
    }

    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteScanData(fp);
    fclose(fp);
    return bRet;
}

bool CLaserMapping::SaveFile()
{
    if(m_aFileSaving.load()){
        return false;
    }
    m_aFileSaving = true;
    if(!WriteLaserParm(m_nStartTime)) {
        m_aFileSaving = false;
        return false;
    }

    if(!WriteScanData()) {
        m_aFileSaving = false;
        return false;
    }
    m_aFileSaving = false;
    return true;
}

bool CLaserMapping::StartMapping()
{
    if (m_bStarted) {
        return true;
    }

    m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);
    std::ifstream FileLocParm(WORK_PATH"RoboLocParm.json");
    Json::Reader Jreader;
    Json::Value LocParmRoot;
    if(!FileLocParm){
        return false;
    }
    if (Jreader.parse(FileLocParm, LocParmRoot)){
        LocParmRoot.toStyledString();
        if (!LocParmRoot["Mapping"]["Resolution"].isNull()) {
            m_dResolution = LocParmRoot["Mapping"]["Resolution"].asDouble();
        }
    }
    FileLocParm.close();

    if (GData::getObj().startRecordDxFlag){
        m_dResolution = 0.05;
    }
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

    // clear the raw scan space
    if(GData::getObj().startRecordDxFlag){
        sensor::CRawScan pRawScan;
        pRawScan.Clear();
    }

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
    if(pthread_create(&m_pMappingThread, &attr, LaserMappingSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat LaserMappingSupportProc Pthread Failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Creat LaserMappingSupportProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr);

    m_bFirstTime = true;
    m_bStarted = true;

    return true;
}

bool CLaserMapping::StopMapping()
{
    if (!m_bStarted)
        return false;

    if(!Stop()) {
        return false;
    }

    if(!SaveFile()) {
        return false;
    }

    return true;
}

void CLaserMapping::SetWorkMode(unsigned char uMode)
{
    m_aWorkMode = uMode;
}


} // namespace mapping

