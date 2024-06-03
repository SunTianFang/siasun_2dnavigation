//                                - CRoboLocClnt.CPP -
//
//   Implementatin of class "CRoboLocClnt".
//

#include "stdafx.h"
#include "RoboLocClnt.h"
#include <fstream>
#include "json/json.h"
#include "Tools.h"
#include "AgvUdpSock.h"
#include "blackboxhelper.hpp"
#include "BaseOdometry.h"
#include "CanMan.h"
#include "AffinePosture.h"
#include "ParameterObject.h"
#include "HttpCommunicationGlobalData.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#if defined USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

#define REUSE_ROBO_INITPOS_TIME     5000
#define ROBO_POSE_TOLERANCE_TIME     500//200ms; By yu : In 500*500 , SlamLocTime >= 500ms.This is temporary,need to cut dowm LocTime.

//////////////////////////////////////////////////////////////////////////////
//   The support routine of Robot localization protocol.
void* RLocClntSupportProc(LPVOID pParam)
{
    CRoboLocClnt* pRLocClnt = reinterpret_cast<CRoboLocClnt*>(pParam);
    while (WaitForSingleObject(pRLocClnt->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
        pRLocClnt->m_LocCritSect.Lock();
        pRLocClnt->SupportRoutine();
        pRLocClnt->m_LocCritSect.Unlock();

        Sleep(RLS_CTRL_CYCLE);
    }

    SetEvent(pRLocClnt->m_hThreadDead);
    pthread_exit(NULL);

    return NULL;
}

//
//   The thread procedure for the deferred call.
//
void* RLocClntRecvThreadProc(LPVOID pParam)
{
    CRoboLocClnt* pRLocClnt = reinterpret_cast<CRoboLocClnt*>(pParam);

    // Loop until the kill event is set
    while (WaitForSingleObject(pRLocClnt->m_hKillRecvThread, 0) != WAIT_OBJECT_0)
    {
        pRLocClnt->m_LocCritSect.Lock();
        pRLocClnt->ProcessReceiveData();
        pRLocClnt->m_LocCritSect.Unlock();

        // Waiting for calling event if this is a "hard" timer
        sem_wait(pRLocClnt->m_hRecvCalling); // Block Wait
    }

    SetEvent(pRLocClnt->m_hRecvThreadDead);
    pthread_exit(NULL);

    return NULL;
}

///////////////////////////////////////////////////////////////////////////////

CRoboLocClnt::CRoboLocClnt(CString strRemoteIp, UINT uRemotePort, BOOL bFramed, BOOL bAscii) :
CUdpChannel(strRemoteIp, uRemotePort, bFramed)
{
    m_bStarted = false;
    m_bUsed = false;

    m_hKillThread = NULL;
    m_hThreadDead = NULL;
    m_pLocClntThread = 0;

    m_hKillRecvThread = NULL;
    m_hRecvThreadDead = NULL;
    m_pLocRecvThread = 0;

    m_hRecvCalling = NULL;
    m_strVersion = "1.0";
    m_uLocalPort = RL_CLIENT_UDP_PORT;
    m_uRemotePort = RL_SERVER_UDP_PORT;

    m_pLocUdpSocket = NULL;
    m_bUseSoftPls = false;

    SetProtocolFormat(bFramed, bAscii);
    SetProtoMask(RLP_CLNT, RLP_SRV);

    Reset();
}

CRoboLocClnt::~CRoboLocClnt()
{
   Stop();
}

//
bool CRoboLocClnt::Initialize()
{
    bool bRet = false;
    bRet = Install();

    // set sync time param
    m_TxLocDataSet.m_SyncTime.bEnable = false;
    m_TxLocDataSet.m_SyncTime.bRemote = false;
    m_TxLocDataSet.m_SyncTime.nCycleTime = RLP_SYNC_TIME_SEND_CYCLE;
    m_TxLocDataSet.m_SyncTime.nTolerateSpan = RLP_SYNC_TIME_TOLERATE_SPAN;
    m_TxLocDataSet.m_SyncTime.nTimeOut = RLP_SYNC_TIME_TIMEOUT;

    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("SyncTime_RobotEnd_Enable", m_TxLocDataSet.m_SyncTime.bEnable);
    pParameterObject->GetParameterValue("SyncTime_RobotEnd_Remote", m_TxLocDataSet.m_SyncTime.bRemote);
    pParameterObject->GetParameterValue("SyncTime_RobotEnd_CycleTime", m_TxLocDataSet.m_SyncTime.nCycleTime);
    pParameterObject->GetParameterValue("SyncTime_RobotEnd_TolerateSpan", m_TxLocDataSet.m_SyncTime.nTolerateSpan);
    pParameterObject->GetParameterValue("SyncTime_RobotEnd_Timeout", m_TxLocDataSet.m_SyncTime.nTimeOut);
    pParameterObject->GetParameterValue("Diagnosis_UseSoftPls", m_bUseSoftPls);

    return bRet;
}

//
//   Install the RLP manager.
//
BOOL CRoboLocClnt::Install()
{
    if(!IsUsed())
    {
        return TRUE;
    }

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return FALSE;

    // Start the support procedure
    pthread_attr_t attr1;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr1);
    if(pthread_create(&m_pLocClntThread, &attr1, RLocClntSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RLocClntSupportProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
    {
        std::cout<<"Creat RLocClntSupportProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr1);


    m_hKillRecvThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    m_hRecvThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    m_hRecvCalling = CreateEvent(NULL, FALSE, FALSE, NULL);

    if (m_hKillRecvThread == NULL || m_hRecvThreadDead == NULL || m_hRecvCalling == NULL)
        return FALSE;

    pthread_attr_t attr2;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr2);
    if(pthread_create(&m_pLocRecvThread, &attr2, RLocClntRecvThreadProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RLocClntRecvThreadProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
    {
        std::cout<<"Creat RLocClntRecvThreadProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr2);

    Reset();

    m_bStarted = true;

    return TRUE;
}

void CRoboLocClnt::Reset()
{
    m_RxLocDataSet.Initialize();
    m_TxLocDataSet.Initialize();
    m_PreRxLocDataSet.Initialize();
	// dq VISION
    m_CamLocDataSet.Initialize();
	
    m_aUseInitPos = false;
    m_PoseDataRecord.Initialize();
    m_SyncTimeData.Initialize();
    m_LastPoseTime = 0;

    m_aCurWorkMode = RLP_MODE_STANDBY;
    m_aNewWorkMode = RLP_MODE_STANDBY;
}

//
BOOL CRoboLocClnt::Create()
{
    std::ifstream FileLocParm(WORK_PATH"RoboLocParm.json");
    Json::Reader Jreader;
    Json::Value LocParmRoot;

    if(!FileLocParm)
    {
        return FALSE;
    }

    if (!Jreader.parse(FileLocParm, LocParmRoot)){
        FileLocParm.close();
        return FALSE;
    }

    // Common parameter
    if (!LocParmRoot["LocVersion"].isNull())
    {
        m_strVersion = LocParmRoot["LocVersion"].asString();
    }
    else
    {
        m_strVersion = "1.0";
    }

    if (!LocParmRoot["LocUse"].isNull())
    {
        m_bUsed = LocParmRoot["LocUse"].asBool();
    }
    else
    {
        m_bUsed = false;
    }

    // Server parameter
    char uchIpBuf[16];
    memset(uchIpBuf, 0, 16 * sizeof(char));

    if (!LocParmRoot["LocServer"]["IP"].isNull())
    {
        unsigned long nCount = LocParmRoot["LocServer"]["IP"].asString().size();
        if (nCount < 16)
        {
            memcpy(uchIpBuf, LocParmRoot["LocServer"]["IP"].asString().c_str(), nCount * sizeof(char));

            SetRemoteIp(uchIpBuf);
            m_pLocUdpSocket->SetRLSrvIp(uchIpBuf);
        }
    }

    if (!LocParmRoot["LocServer"]["Port"].isNull()){
        m_uRemotePort = LocParmRoot["LocServer"]["Port"].asUInt();
        SetRemotePort(m_uRemotePort);
    }

    // client parameter,默认使用第一个客户端配置参数
    int nClientCount = 0;
    if (!LocParmRoot["LocClient"].isNull()) {
        nClientCount = LocParmRoot["LocClient"].size();
    }
    for(int j = 0; j < nClientCount; j++){
        if (!LocParmRoot["LocClient"][j]["Port"].isNull()){
            m_uLocalPort = LocParmRoot["LocClient"][j]["Port"].asUInt();
        }
    }

    // net proto
    std::string proto_version = "";
    if (!LocParmRoot["NetProto"]["Version"].isNull()){
        proto_version = LocParmRoot["NetProto"]["Version"].asString();
        SetProtoVersion(proto_version);
    }

    bool bFramed = false;
    bool bAscii = false;
    bool bRet[2] = {false};
    if (!LocParmRoot["NetProto"]["Format"]["Framed"].isNull()){
        bFramed = LocParmRoot["NetProto"]["Format"]["Framed"].asBool();
        bRet[0] = true;
    }
    else {
        bRet[0] = false;
    }
    if (!LocParmRoot["NetProto"]["Format"]["Ascii"].isNull()){
        bAscii = LocParmRoot["NetProto"]["Format"]["Ascii"].asBool();
        bRet[1] = true;
    }
    else {
        bRet[1] = false;
    }
    if(bRet[0] && bRet[1]){
        SetProtocolFormat(bFramed, bAscii);
    }

    unsigned char server_mask = 0;
    unsigned char client_mask = 0;
    if (!LocParmRoot["NetProto"]["Header"]["ServerMask"].isNull()){
        memcpy(&server_mask, LocParmRoot["NetProto"]["Header"]["ServerMask"].asString().c_str(), 1 * sizeof(unsigned char));
        bRet[0] = true;
    }
    else {
        bRet[0] = false;
    }
    if (!LocParmRoot["NetProto"]["Header"]["ClientMask"].isNull()){
        memcpy(&client_mask, LocParmRoot["NetProto"]["Header"]["ClientMask"].asString().c_str(), 1 * sizeof(unsigned char));
        bRet[1] = true;
    }
    else {
        bRet[1] = false;
    }
    if(bRet[0] && bRet[1]){
        SetProtoMask(client_mask, server_mask);
    }

    unsigned char server_id = 0;
    unsigned char client_id = 0;
    if (!LocParmRoot["NetProto"]["Header"]["ServerId"].isNull()){
        server_id = LocParmRoot["NetProto"]["Header"]["ServerId"].asUInt();
        bRet[0] = true;
    }
    else {
        bRet[0] = false;
    }
    if (!LocParmRoot["NetProto"]["Header"]["ClientId"].isNull()){
        client_id = LocParmRoot["NetProto"]["Header"]["ClientId"].asUInt();
        bRet[1] = true;
    }
    else {
        bRet[1] = false;
    }
    if(bRet[0] && bRet[1]){
        SetProtoID(client_id, server_id);
    }

    FileLocParm.close();
    return TRUE;
}

bool CRoboLocClnt::Stop()
{
    if (!m_bStarted)
        return FALSE;

    if (m_pLocUdpSocket != NULL)
    {
       delete m_pLocUdpSocket;
        m_pLocUdpSocket = NULL;
    }

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
    PthreadJoin(m_pLocClntThread);

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


    SetEvent(m_hKillRecvThread);
    SetEvent(m_hRecvCalling);

    WaitForSingleObject(m_hRecvThreadDead, 5000);
    PthreadJoin(m_pLocRecvThread);

    if (m_hKillRecvThread != NULL)
    {
        CloseHandle(m_hKillRecvThread);
        m_hKillRecvThread = NULL;
    }

    if (m_hRecvThreadDead != NULL)
    {
        CloseHandle(m_hRecvThreadDead);
        m_hRecvThreadDead = NULL;
    }

    if (m_hRecvCalling != NULL)
    {
        CloseHandle(m_hRecvCalling);
        m_hRecvCalling = NULL;
    }

    m_bStarted = false;

    return TRUE;
}

//
bool CRoboLocClnt::CreateUdpCom()
{
    bool bResult = false;

    if (m_pLocUdpSocket == NULL)
    {
        m_pLocUdpSocket = new CAgvUdpSocket;
    }

    if(!m_pLocUdpSocket)
    {
        return false;
    }

    if(!Create())
    {
        return false;
    }

    bResult = m_pLocUdpSocket->Create(m_uLocalPort, MASTER_UDP_PORT, MAINTAIN_UDP_PORT);
    if (bResult)
    {
        m_pLocUdpSocket->SetRLSrvPort(m_uRemotePort);
        m_pLocUdpSocket->SetRLPChannel(this);

        SetSocket(m_pLocUdpSocket);

        bResult = Initialize();
    }

    return bResult;
}

//
//   The support routine of the network manager.
//
void CRoboLocClnt::SupportRoutine()
{
   SupportAutoPosAgent();

   ProcessNetStatus();

   ProcessDataSend();

   //更新同步时间缓存数据
   {
       std::lock_guard<std::mutex> lock(sync_time_mtx);
       m_SyncTimeData = m_TxLocDataSet.m_SyncTime;
   }
}

// dq VISION
void CRoboLocClnt::ProcessReceiveData()
{
    char chDummy = 0;
    UCHAR uType = 0;
    bool bRet = false;
    //this->SetProtocolFormat(false,true);
	while(RxReady())
    {
		int nRecvCount = GetRecvCount();
        if(nRecvCount > 100 ||nRecvCount ==78)
        {
            while (RxReady())
                *this >> chDummy;
            break;
        }
        //std::cout<<"\033[33;1mm_nRecFromCam: "<<m_nRecFromCam<<", nRecvCount: "<<nRecvCount<<"\033[0m"<<std::endl;
        if (!GetRecvFromCam())
        {
            // 对数据进行预处理
            if (!Preprocess())
            {
    #if defined USE_BLACK_BOX
                FILE_BlackBox(LocBox,"-->Preprocess is failed! nRecvCount=", nRecvCount);
    #endif
                break;
            }

            if (!GetRxHeader())
            {
    #if defined USE_BLACK_BOX
                FILE_BlackBox(LocBox,"-->GetRxHeader is failed! nRecvCount=", nRecvCount);
    #endif
                std::cout<<"-->GetRxHeader is failed! nRecvCount="<<std::endl;
                continue;
            }

            *this >> uType;

            switch(uType)
            {
            case RLP_SRV_GET_LOCINFO:
            {
                bRet = SrvGetLocInfoDe(this, m_RxLocDataSet.m_LocInfo);
                if(bRet)
                {
                    ReportLocInfo();
                }
            }
                break;

            case RLP_SRV_SET_ATTR:
            {
                bRet = SrvSetAttrDe(this, m_RxLocDataSet.m_LocAttr);
                if(bRet)
                {
                    SetLocAttr();
                }
            }
                break;

            case RLP_SRV_GET_ATTR:
            {
                bRet = SrvGetAttrDe(this, m_RxLocDataSet.m_LocAttr);
                if(bRet)
                {
                    ReportLocAttr();
                }
            }
                break;

            case RLP_SRV_GET_SYNC_TIME:
            {
                bRet = SrvGetSyncTimeDe(this, m_RxLocDataSet.m_SyncTime);
                if(bRet)
                {
                    m_TxLocDataSet.m_SyncTime.lSrvTime = m_RxLocDataSet.m_SyncTime.lSrvTime;
                    m_TxLocDataSet.m_SyncTime.lClntTime = GetTickCount();
                    ClntReportSyncTimeSe(this, m_TxLocDataSet.m_SyncTime);
                }
            }
                break;

            case RLP_SRV_REPORT_SYNC_TIME:
                bRet = SrvReportSyncTimeDe(this, m_RxLocDataSet.m_SyncTime);
                if(bRet){
                    HandleSyncTimeFrame();
                }
                break;

            case RLP_SRV_SET_MODE:
            {
                bRet = SrvSetModeDe(this, m_RxLocDataSet.m_OperateModeW);
                if(bRet)
                {
                    ChangeMode();
                }
            }
                break;

            case RLP_SRV_GET_MODE:
            {
                bRet = SrvGetModeDe(this, m_RxLocDataSet.m_OperateModeR);
                if(bRet)
                {
                    ReportLocMode();
                }
            }
                break;

            case RLP_SRV_SET_POSITION:
            {
                bRet = SrvSetPositionDe(this, m_RxLocDataSet.m_PositionData);
                if(bRet)
                {
                    SetRoboPosition();
                }
            }
                break;

            case RLP_SRV_GET_POSE:
            {
                bRet = SrvGetPoseDe(this, m_RxLocDataSet.m_VelocityData);
                //std::cout<<"\033[33;1mbRet   : "<<bRet<<"\033[0m"<<std::endl;

                if(bRet){
                    auto pBaseOdom = BaseOdomSingleton::GetInstance();
                    STVelocityData& velRx_ = m_RxLocDataSet.m_VelocityData;
                    unsigned long long timeNow = GetTickCount();
                    // dq VISION report Vel to Cam
                    bool bSendVelMsgtoCam = ClntReportVel_Cam(this, m_RxLocDataSet.m_VelocityData);
                    // update the laser odometry according to the velocity vector.
                    pBaseOdom->UpdateVelocity(velRx_.Vx, velRx_.Vy, velRx_.Vtheta, velRx_.SteerAngle, velRx_.HeadingAngle,
                                              velRx_.GyroAngle, velRx_.lGyroTime, velRx_.lRawTime, velRx_.coordBase, velRx_.lRecvTime);
    #if defined USE_BLACK_BOX
                    FILE_BlackBox(LocBox,"SetVel:", velRx_.Vx,",", velRx_.Vy, ",", velRx_.Vtheta, ",",velRx_.SteerAngle,
                                  "," ,velRx_.HeadingAngle, ",", velRx_.GyroAngle,",", (int)velRx_.lGyroTime, ",",
                                  (int)velRx_.lRawTime, ",", (int)velRx_.lRecvTime,",",(int)velRx_.Wait,",",(int)velRx_.Mask,",",(int)timeNow);
    #endif
                    OnServerEcho();
                }
                else
                {
                    auto pBaseOdom = BaseOdomSingleton::GetInstance();
                    STVelocityData& velRx_ = m_RxLocDataSet.m_VelocityData;
                    unsigned long long timeNow = GetTickCount();
#if defined USE_BLACK_BOX
                FILE_BlackBox(LocBox,"????????Err_SetVel:", velRx_.Vx,",", velRx_.Vy, ",", velRx_.Vtheta, ",",velRx_.SteerAngle,
                              "," ,velRx_.HeadingAngle, ",", velRx_.GyroAngle,",", (int)velRx_.lGyroTime, ",",
                              (int)velRx_.lRawTime, ",", (int)velRx_.lRecvTime,",",(int)velRx_.Wait,",",(int)velRx_.Mask,",",(int)timeNow);
#endif
                }
            }
                break;

            case RLP_SRV_SET_PLS_LAYER:
            {
                bRet = SrvSetPlsLayerDe(this, m_RxLocDataSet.m_PlsLayer);
                //printf("ip %s,%d\n",m_RxLocDataSet.m_PlsLayer.laserIp.c_str(),m_RxLocDataSet.m_PlsLayer.chSetLayer);
                if(bRet)
                {
                    bool needPush = true;
                    for(int i = 0; i < pslLayerData.size();i++)
                    {
                        if(m_RxLocDataSet.m_PlsLayer.laserIp == pslLayerData[i].laserIp)
                        {
                            pslLayerData[i].chSetLayer = m_RxLocDataSet.m_PlsLayer.chSetLayer;
                            pslLayerData[i].iKey = m_RxLocDataSet.m_PlsLayer.iKey;
                            needPush = false;
                        }
                    }
                    if(needPush)
                    {
                        pslLayerData.push_back(m_RxLocDataSet.m_PlsLayer);
                    }
                }
            }
                break;
            case RLP_SRV_SEND_HEARTBEAT:
            {
                bRet = SrvSendHeartBeatDe(this, m_RxLocDataSet.m_HeartBeat);
                if(bRet)
                {
                    HandleHeartBeatFrame();
                }
            }
                break;

            default:
    #if defined USE_BLACK_BOX
                FILE_BlackBox(LocBox,"--> Master sends error command: ", uType);
    #endif
                uType = RLP_CLNT_COMMAND_ERROR;

                if(!IsAscii())
                {
                    while (RxReady())
                        *this >> chDummy;
                }
                break;
            }
        }

        else if (GetRecvFromCam())
        {
            if(!GetRxHeaderLANXIN())
            {
                // dq VISION
                std::cout<<"\033[31;1m----GetRxHeader is failed!!!----\033[0m"<<nRecvCount<<std::endl;
                continue;
            }

            (*this->GetChannelObject()) >> uType;

            switch(uType)
            {
            case RLP_CLNT_RECV_POS_CAM :
            {
                bRet = ClntGetPos_Cam(this, m_CamLocDataSet.m_PositionData);
                if(bRet)
                {
                    SaveLocPosition_Cam();
                }
            }
                break;

            case RLP_CLNT_RECV_RELOCPOS_CAM:
            {
                bRet = ClntGetRelocPos_Cam(this, m_CamLocDataSet.m_PositionData);
                if(bRet)
                {
                    SaveLocPosition_Cam();
                }
            }
                break;

            case RLP_CLNT_RECV_LOCMODE_CAM:
            {
                bRet = ClntGetMode_Cam(this,m_LocMode_Cam);
                // std::cout<<"ClntGetMode_Cam(this,m_LocMode_Cam): "<<m_LocMode_Cam<<std::endl;
            }
                break;

            }
        }
    }
}

// 判断网络通讯状态
void CRoboLocClnt::ProcessNetStatus()
{
    unsigned long long tmNow = GetTickCount();     // Get the current time

    // 数据交互超时,认为通讯中断
    if (!m_RxLocDataSet.m_bNetBlocked && (tmNow - m_RxLocDataSet.m_tLastEcho > RLP_NET_COMM_TIMEOUT))
    {
        m_RxLocDataSet.m_bNetBlocked = true;

#if defined USE_BLACK_BOX
        FILE_BlackBox(LocBox,"--> Master Network blocked!");
#endif
    }

    if (m_RxLocDataSet.m_bNetBlocked)
    {
        // 发送PING指令
    }
}

//
void CRoboLocClnt::ProcessDataSend()
{
    if (m_RxLocDataSet.m_bNetBlocked){
        return;
    }

    if(m_aCurWorkMode.load() == RLP_MODE_LOCALIZATION || m_aCurWorkMode.load() == RLP_MODE_MAPPING){
        // 周期发送同步时间数据帧
        if(m_TxLocDataSet.m_SyncTime.bEnable && m_TxLocDataSet.m_SyncTime.bRemote){
            static bool sync_blocked_record = false;
            unsigned long long tmNow = GetTickCount();
            unsigned int   sync_cycle_time = 0;
            m_TxLocDataSet.m_SyncTime.lClntTime = tmNow;

            // 同步失败时,高频发送时间同步帧;同步成功时,低频发送时间同步帧.
            if(SyncTimeBlocked()){
                sync_cycle_time = 3000; //3s
            }
            else if(sync_blocked_record){
                sync_cycle_time = 0;
            }
            else{
                sync_cycle_time = m_TxLocDataSet.m_SyncTime.nCycleTime;
            }

            if(tmNow - m_TxLocDataSet.m_SyncTime.timeLastSend > sync_cycle_time){
                ClntGetSyncTimeSe(this, m_TxLocDataSet.m_SyncTime);
                m_TxLocDataSet.m_SyncTime.timeLastSend = tmNow;
            }

            sync_blocked_record = SyncTimeBlocked();
        }
    }


    // 周期发送定位位姿
    ReportRoboPose();
    // 周期报告pls状态
    if(m_bUseSoftPls)
        ReportPlsState();
}

// Support auto-positioning
void CRoboLocClnt::SupportAutoPosAgent()
{
    switch(m_aCurWorkMode.load())
    {
    case RLP_MODE_POWERON:
        break;

    case RLP_MODE_STANDBY:
        LocStandBy();
        break;

    case RLP_MODE_MAPPING:
        DoMapping();
        break;

    case RLP_MODE_LOCALIZATION:
        DoLocalization();
        break;

    case RLP_MODE_SCAN:
        DoScan();
        break;

    default:
        break;
    }
}

// 上报定位系统信息
bool CRoboLocClnt::ReportLocInfo()
{
    // 获取定位系统相关信息

    //报给车体的设备ID和软件版本号
    m_TxLocDataSet.m_LocInfo.ID = 0;
    m_TxLocDataSet.m_LocInfo.nSwVersion = 20010;

    ClntReportLocInfoSe(this, m_TxLocDataSet.m_LocInfo);

    return true;
}

// 设置定位系统属性数据
bool CRoboLocClnt::SetLocAttr()
{
    // 设置定位系统相关属性


//    m_RxLocDataSet.m_LocAttr.nRadiusFrom;
//    m_RxLocDataSet.m_LocAttr.nRadiusTo;
//    m_RxLocDataSet.m_LocAttr.NClosest;


    ClntReplyAttrSe(this, m_RxLocDataSet.m_LocAttr);

    return true;
}

// 上报定位系统属性数据
bool CRoboLocClnt::ReportLocAttr()
{
    // 获取定位系统相关属性


    //    m_TxLocDataSet.m_LocAttr.nRadiusFrom;
    //    m_TxLocDataSet.m_LocAttr.nRadiusTo;
    //    m_TxLocDataSet.m_LocAttr.NClosest;

    ClntReportAttrSe(this, m_TxLocDataSet.m_LocAttr);

    return true;
}

// 切换定位系统工作模式
bool CRoboLocClnt::ChangeMode()
{
    bool bRet = false;

    STOperateMode& opModeTx_ = m_TxLocDataSet.m_OperateModeW;
    STOperateMode& opModeRx_ = m_RxLocDataSet.m_OperateModeW;

    if(GData::getObj().startRecordDxFlag){
        m_aNewWorkMode = RLP_MODE_MAPPING;
    }
    else if(GData::getObj().finishRecordDxFlag){
        m_aNewWorkMode = RLP_MODE_LOCALIZATION;
    }
    else{
        m_aNewWorkMode = opModeRx_.chMode;
    }

    if(m_aCurWorkMode.load() != m_aNewWorkMode.load()){
        // 更改成新模式
        switch(m_aNewWorkMode.load())
        {
        case RLP_MODE_POWERON:
            bRet = true;
            break;

        case RLP_MODE_STANDBY:
            bRet = EnterStandByMode();
            break;

        case RLP_MODE_MAPPING:
            bRet = EnterMappingMode();
            break;

        case RLP_MODE_LOCALIZATION:
            bRet = EnterLocalizationMode();
            break;

        case RLP_MODE_SCAN:
            bRet = EnterScanMode();
            break;
        case RLP_MODE_CALIBRATION:

            break;
        default:
            break;
        }

        /*if(bRet){
            m_aCurWorkMode = m_aNewWorkMode.load();
            opModeTx_.chErrCode = 0;
        }
        else{
            opModeTx_.chErrCode = 1;
        }*/
    }
    else{
        opModeTx_.chErrCode = 0;
    }

    if(GData::getObj().finishRecordDxFlag){
        GData::getObj().finishRecordDxFlag = false;
    }

    opModeTx_.chMode = m_aCurWorkMode.load();
    ClntReplyModeSe(this, opModeTx_);

#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox,"Client set new work mode: ", (int)m_aNewWorkMode.load());
#endif

    return bRet;
}

// 上报定位系统工作模式
bool CRoboLocClnt::ReportLocMode()
{
    // 上报定位系统当前工作模式
    m_TxLocDataSet.m_OperateModeR.chMode = m_aCurWorkMode.load();

    ClntReportModeSe(this, m_TxLocDataSet.m_OperateModeR);

#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox,"Report work mode: ", (int)m_aCurWorkMode.load());
#endif

    return true;
}

// 设置机器人当前位姿
bool CRoboLocClnt::SetRoboPosition()
{
    STPositionData posDataRx_;
    STPositionData posDataTx_;
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        posDataRx_ = m_RxLocDataSet.m_PositionData;
        posDataTx_ = m_TxLocDataSet.m_PositionData;
    }

    if(m_aCurWorkMode.load() == RLP_MODE_LOCALIZATION){
        std::lock_guard<std::mutex> lock(data_mtx);
        // 设置定位系统的当前位姿，如果成功　uErrCode=0, 否则 uErrCode=1;
        if(m_PreRxLocDataSet.m_PositionData != posDataRx_) {
            m_PreRxLocDataSet.m_PositionData = posDataRx_;
            m_aUseInitPos = true;
        }
        else {
            // if(posDataRx_.lRawTime - m_PreRxLocDataSet.m_PositionData.lRawTime > REUSE_ROBO_INITPOS_TIME && !IsBlocked()) {
            //   m_PreRxLocDataSet.m_PositionData = posDataRx_;
            //   m_aUseInitPos = true;
            // }
        }
        posDataTx_.uErrCode = 0;
    }
    else{
        posDataTx_.uErrCode = 2; // 工作模式不匹配
    }

    posDataTx_.chMapId = posDataRx_.chMapId;
    posDataTx_.nX = posDataRx_.nX;
    posDataTx_.nY = posDataRx_.nY;
    posDataTx_.nTheta = posDataRx_.nTheta;
    posDataTx_.lRawTime = posDataRx_.lRawTime;
    posDataTx_.uTimeSpan = posDataRx_.uTimeSpan;

    ClntReplyPositionSe(this, posDataTx_);

#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox, "SetRoboPos:", posDataRx_.nX,",", posDataRx_.nY, ",", posDataRx_.nTheta,
                  ",", (int)posDataRx_.lRawTime, "," ,posDataRx_.uTimeSpan, ",", (int)m_aUseInitPos.load(), ",",posDataTx_.uErrCode);
#endif

    return true;
}

// 上报机器人位姿
bool CRoboLocClnt::ReportRoboPose()
{
    if(!(m_aCurWorkMode.load() == RLP_MODE_LOCALIZATION || m_aCurWorkMode.load() == RLP_MODE_MAPPING)){
        return false;
    }

    unsigned long long cur_pose_time = 0;
    STVelocityData velDataRx_;
    STPoseData poseDataTx_;
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        velDataRx_ = m_RxLocDataSet.m_VelocityData;
        poseDataTx_ = m_TxLocDataSet.m_PoseData;
        cur_pose_time = poseDataTx_.lRawTime;
    }

    if(m_aCurWorkMode.load() == RLP_MODE_LOCALIZATION){
        poseDataTx_.uTimeSpan = 0;
    }
    else if (m_aCurWorkMode.load() == RLP_MODE_MAPPING) {
        poseDataTx_.uTimeSpan = 0;
    }
    else{
        poseDataTx_.uErrCode = RLP_WRONG_OPERATE_MODE; // 工作模式不匹配
    }

    // 同步时间超时
    if(SyncTimeBlocked()){
        poseDataTx_.uErrCode = RLP_ROBOT_SYNCTIME_FAILED;
    }

    //导航控制器和车体主控制器通信中断
    if(IsBlocked()) {
        poseDataTx_.uErrCode = RLP_ROBOT_TIMEOUT;
    }

    poseDataTx_.chMask = velDataRx_.Mask;
    poseDataTx_.chWait = velDataRx_.Wait;
    poseDataTx_.GyroAngle = 0.0;
    poseDataTx_.lGyroTime = 0;

    // 如果陀螺仪在本地,需要把陀螺仪角度上报给机器人
    auto pBaseOdom = BaseOdomSingleton::GetInstance();
    auto pCanManager = CanManagerSingleton::GetInstance();
    CFloatDataInfo gyro_angle;
    gyro_angle.clear();
    if(pCanManager != NULL && pCanManager->m_bActivation && !pBaseOdom->m_bGyroRemote){
        if(pCanManager->GetRelAngle(gyro_angle)){
            poseDataTx_.GyroAngle = static_cast<int>(gyro_angle.m_fData * 1000);
            poseDataTx_.lGyroTime = gyro_angle.m_dwTimeStamp;
        }
        else {
            //poseDataTx_.GyroAngle = 0;
            poseDataTx_.lGyroTime = 0;
            poseDataTx_.uErrCode = RLP_GYRO_TIMEOUT;    //本地陀螺仪通信中断
        }
    }

    // 根据里程计和时间戳进行位姿插补，以保证发送时的位姿是准确的
    unsigned long long cur_time = GetTickCount();
    int time_span = 0;
    if(m_aCurWorkMode.load() == RLP_MODE_LOCALIZATION){
        time_span = static_cast<int>(cur_time - cur_pose_time);
        if((cur_pose_time > m_LastPoseTime) || (time_span > 0 && time_span < ROBO_POSE_TOLERANCE_TIME)){
            //位姿插补
            CPosture est_pose;
            Eigen::Affine3d tran_estimate;
            est_pose.x = static_cast<double>(poseDataTx_.nX) / 1000.0;
            est_pose.y = static_cast<double>(poseDataTx_.nY) / 1000.0;
            est_pose.fThita = static_cast<double>(poseDataTx_.nTheta) / 1000.0;
            tran_estimate = PostureToAffine(est_pose.x, est_pose.y, est_pose.fThita);

            CPosture odomTrans;
            auto pOdometry = BaseOdomSingleton::GetInstance();
            pOdometry->GetLocalOdomTrans(cur_pose_time, cur_time, odomTrans);
            Eigen::Affine3d tran_odom = PostureToAffine(odomTrans);
            tran_estimate = tran_estimate * tran_odom;

            CPosture new_pose = AffineToPosture(tran_estimate);
            poseDataTx_.nX = static_cast<int>(new_pose.x * 1000.0);
            poseDataTx_.nY = static_cast<int>(new_pose.y * 1000.0);
            poseDataTx_.nTheta = static_cast<int>(new_pose.fThita * 1000.0);
            poseDataTx_.lRawTime = cur_time;

//            // By Sam Add For LegMethod
//            CPosture est_legpose;
//            Eigen::Affine3d leg_tran_estimate;
//            est_legpose.x = static_cast<double>(poseDataTx_.nLegX) / 1000.0;
//            est_legpose.y = static_cast<double>(poseDataTx_.nLegY) / 1000.0;
//            est_legpose.fThita = static_cast<double>(poseDataTx_.nLegTheta) / 1000.0;
//            leg_tran_estimate = PostureToAffine(est_legpose);

//            //leg_tran_estimate = leg_tran_estimate * tran_odom; // no need add tran_odom

//            CPosture new_leg_pose = AffineToPosture(leg_tran_estimate);
//            poseDataTx_.nLegX = static_cast<int>(new_leg_pose.x * 1000.0);
//            poseDataTx_.nLegY = static_cast<int>(new_leg_pose.y * 1000.0);
//            poseDataTx_.nLegTheta = static_cast<int>(new_leg_pose.fThita * 1000.0);

#ifdef USE_LEG_METHOD
            CPosture leg_diff_pose;
            Eigen::Affine3d leg_diff_affine;
            leg_diff_pose.x = static_cast<double>(poseDataTx_.nDiffLegX) / 1000.0;
            leg_diff_pose.y = static_cast<double>(poseDataTx_.nDiffLegY) / 1000.0;
            leg_diff_pose.fThita = static_cast<double>(poseDataTx_.nDiffLegTheta) / 1000.0;
            leg_diff_affine = PostureToAffine(leg_diff_pose.x, leg_diff_pose.y, leg_diff_pose.fThita);
            leg_diff_affine = leg_diff_affine * tran_odom.inverse();

            CPosture new_leg_diff_pose = AffineToPosture(leg_diff_affine);
            poseDataTx_.nDiffLegX = static_cast<int>(new_leg_diff_pose.x * 1000.0);
            poseDataTx_.nDiffLegY = static_cast<int>(new_leg_diff_pose.y * 1000.0);
            poseDataTx_.nDiffLegTheta = static_cast<int>(new_leg_diff_pose.fThita * 1000.0);
#endif
        }
        else if (time_span > ROBO_POSE_TOLERANCE_TIME) {
            //位姿数据超时,用上次发送的位姿
            if(m_PoseDataRecord.lRawTime > poseDataTx_.lRawTime){
                poseDataTx_.nX = m_PoseDataRecord.nX;
                poseDataTx_.nY = m_PoseDataRecord.nY;
                poseDataTx_.nTheta = m_PoseDataRecord.nTheta;
                poseDataTx_.uG = m_PoseDataRecord.uG;
                poseDataTx_.uN = m_PoseDataRecord.uN;
                poseDataTx_.chPosMode = m_PoseDataRecord.chPosMode;
                poseDataTx_.lRawTime = m_PoseDataRecord.lRawTime;

#ifdef USE_LEG_METHOD
                // By Sam Add For LegMethod
                poseDataTx_.nLegX = m_PoseDataRecord.nLegX;
                poseDataTx_.nLegY = m_PoseDataRecord.nLegY;
                poseDataTx_.nLegTheta = m_PoseDataRecord.nLegTheta;
                poseDataTx_.nDiffLegX = m_PoseDataRecord.nDiffLegX;
                poseDataTx_.nDiffLegY = m_PoseDataRecord.nDiffLegY;
                poseDataTx_.nDiffLegTheta = m_PoseDataRecord.nDiffLegTheta;
#endif
            }
        }
        else {
            //当前位姿是最新的
        }
    }
    m_PoseDataRecord = poseDataTx_;
    m_LastPoseTime = cur_pose_time;

    ClntReportPoseSe(this, poseDataTx_);

#ifdef USE_BLACK_BOX
    #ifdef USE_LEG_METHOD
        FILE_BlackBox(LocBox, "ReportPose:",poseDataTx_.nX,",",poseDataTx_.nY,",",poseDataTx_.nTheta,
                      ", GoodsCPose: (",poseDataTx_.nLegX,", ", poseDataTx_.nLegY,", ", poseDataTx_.nLegTheta, "), diff_agv_leg: (",
                      poseDataTx_.nDiffLegX,", ", poseDataTx_.nDiffLegY,", ", poseDataTx_.nDiffLegTheta, "), "
                      ,poseDataTx_.uG,",",poseDataTx_.uN,",",(int)cur_pose_time,",",(int)poseDataTx_.lRawTime,","
                      ,(int)cur_time,",",poseDataTx_.uErrCode,",",poseDataTx_.uLegErrCode,",",time_span);
    #else
        FILE_BlackBox(LocBox, "ReportPose:",poseDataTx_.nX,",",poseDataTx_.nY,",",poseDataTx_.nTheta,","
                  ,poseDataTx_.uG,",",poseDataTx_.uN,",",(int)cur_pose_time,",",(int)poseDataTx_.lRawTime,","
                  ,(int)cur_time,",",poseDataTx_.uErrCode,",",time_span);
    #endif
#endif

    return true;
}

// 激光切区设置
bool CRoboLocClnt::SetPlsLayer()
{
    bool bRet = false;
    STPlsLayer& plsLayerRx_ = m_RxLocDataSet.m_PlsLayer;

    return bRet;
}

// 激光pls状态报告
bool CRoboLocClnt::ReportPlsState()
{
    bool bRet = false;
    std::lock_guard<std::mutex> lock(pls_data_mtx);
    STPlsData plsDataTx_;
    plsDataTx_.iLaserNum = pslData.size();
    if(pslData.size() > 0)
    {
        plsDataTx_.vBaseData = pslData;
        //printf("m_TxLocDataSet.m_PlsData state is %d\n",plsDataTx_.vBaseData[0].chState);
        ClntReportPlsStateSe(this,plsDataTx_);
    }
    return bRet;
}

// 处理心跳数据帧
bool CRoboLocClnt::HandleHeartBeatFrame()
{
    unsigned long long tmNow = GetTickCount();

    // 发送心跳数据
    m_TxLocDataSet.m_HeartBeat.lSrvTime = m_RxLocDataSet.m_HeartBeat.lSrvTime;
    m_TxLocDataSet.m_HeartBeat.lClntTime = tmNow;

    ClntReportHeartBeatSe(this, m_TxLocDataSet.m_HeartBeat);

    if(m_RxLocDataSet.m_HeartBeat.lClntTime == 0)
    {
        m_RxLocDataSet.m_HeartBeat.lClntTime = tmNow;
    }

    if(1 /*tmNow - m_RxLocDataSet.m_HeartBeat.lClntTime < RLP_NET_RECV_TIMEOUT*/)
    {
        OnServerEcho();
    }

    return true;
}

// 处理同步时间数据帧
bool CRoboLocClnt::HandleSyncTimeFrame()
{
    if(!m_TxLocDataSet.m_SyncTime.bEnable || !m_TxLocDataSet.m_SyncTime.bRemote){
        return false;
    }
    unsigned long long tmNow = GetTickCount();
    unsigned long long time_span = tmNow - m_RxLocDataSet.m_SyncTime.lClntTime;
    if(time_span < m_TxLocDataSet.m_SyncTime.nTolerateSpan){
        m_TxLocDataSet.m_SyncTime.lTimeOffset = static_cast<long long>(m_RxLocDataSet.m_SyncTime.lClntTime + time_span / 2 - m_RxLocDataSet.m_SyncTime.lSrvTime);
        m_TxLocDataSet.m_SyncTime.uFlag = 1;
        m_TxLocDataSet.m_SyncTime.lCalibTime = tmNow;

#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox, "SyncTime:",(int)tmNow,",", (int)time_span,",", (int)m_TxLocDataSet.m_SyncTime.lTimeOffset,
                  ",", (int)m_TxLocDataSet.m_SyncTime.uFlag,",", (int)m_TxLocDataSet.m_SyncTime.lCalibTime
                  ,",",(int)m_RxLocDataSet.m_SyncTime.lClntTime,",",(int)m_RxLocDataSet.m_SyncTime.lSrvTime);
#endif
    }
    else {
    }

    return true;
}

// 空闲
bool CRoboLocClnt::LocStandBy()
{

    return true;
}

// 执行建图过程
bool CRoboLocClnt::DoMapping()
{

    return true;
}

// 执行定位过程
bool CRoboLocClnt::DoLocalization()
{

    return true;
}

// 执行扫描过程
bool CRoboLocClnt::DoScan()
{

    return true;
}

// 进入空闲模式
bool CRoboLocClnt::EnterStandByMode()
{
    return false;
}

// 进入建图模式
bool CRoboLocClnt::EnterMappingMode()
{
    return false;
}

// 进入定位模式
bool CRoboLocClnt::EnterLocalizationMode()
{
    return false;
}

// 进入扫描模式
bool CRoboLocClnt::EnterScanMode()
{
    return false;
}

// 接收到服务器端的应答
bool CRoboLocClnt::OnServerEcho()
{
    m_RxLocDataSet.m_tLastEcho = GetTickCount();
    m_RxLocDataSet.m_bNetBlocked = false;
    return true;
}

// 同步时间超时阻塞
bool CRoboLocClnt::SyncTimeBlocked()
{
    if(!m_TxLocDataSet.m_SyncTime.bEnable){
        return false;
    }

    if(m_TxLocDataSet.m_SyncTime.bRemote){
        long long time_span = static_cast<long long>(GetTickCount() - m_TxLocDataSet.m_SyncTime.lCalibTime);
        if(labs(time_span) < m_TxLocDataSet.m_SyncTime.nTimeOut && m_TxLocDataSet.m_SyncTime.uFlag == 1){
            return false;
        }
        else {
            m_TxLocDataSet.m_SyncTime.uFlag = 0;
            return true;
        }
    }
    else {
        return false;
    }
}

// 网络通讯是否堵塞.
bool CRoboLocClnt::IsBlocked()
{
    return m_RxLocDataSet.m_bNetBlocked;
}

bool CRoboLocClnt::GetLocalVel(short& Vx, short& Vy, short& Vtheta)
{
    // 临时简单处理
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        STVelocityData& velDataRx_ = m_RxLocDataSet.m_VelocityData;
        Vx = velDataRx_.Vx;
        Vy = velDataRx_.Vy;
        Vtheta = velDataRx_.Vtheta;
    }
    return true;
}
int CRoboLocClnt::GetReflectChargeFlag()
{

        std::lock_guard<std::mutex> lock(data_mtx);
        return m_RxLocDataSet.m_VelocityData.uReserve1;

}

bool CRoboLocClnt::GetRoboInitPosition(unsigned char& chMapId, double& fX, double& fY, double& fTheta, unsigned long long& lRawTime, short& setpose_bypad)
{
    if(m_aUseInitPos.load()) {
        std::lock_guard<std::mutex> lock(data_mtx);
        chMapId = m_PreRxLocDataSet.m_PositionData.chMapId;
        fX = static_cast<double>(m_PreRxLocDataSet.m_PositionData.nX) / 1000.0;
        fY = static_cast<double>(m_PreRxLocDataSet.m_PositionData.nY) / 1000.0;
        fTheta = static_cast<double>(m_PreRxLocDataSet.m_PositionData.nTheta) / 1000.0;
        setpose_bypad = m_PreRxLocDataSet.m_PositionData.uReserve1;
        std::cout<<"Recieved setpose_bypad!!!!!!!!!!  "<< setpose_bypad<<std::endl;
        short uTimeSpan_ = m_PreRxLocDataSet.m_PositionData.uTimeSpan;
        long long pos_raw_time = static_cast<long long>(m_PreRxLocDataSet.m_PositionData.lRawTime);
        unsigned long long cur_pos_time = 0;


        // dq VISION 设置视觉重定位位姿
        if(setpose_bypad ==1)
        {
            std::cout<<"--------------------------------------Set RoboInitPosition Cam!!!--------------------------------------------"<<std::endl;
            SetInitPosition_Cam(m_PreRxLocDataSet.m_PositionData,0);
        }

        //　同步时间机制;获取初始位姿的地方需要根据时间进行插补
        unsigned long long tmNow_ = GetTickCount();
        STSyncTime sync_time_data;
        bool sync_ok = false;
        sync_ok = GetSyncTimeData(sync_time_data);

        if(sync_ok){
            cur_pos_time = static_cast<unsigned long long>(pos_raw_time + sync_time_data.lTimeOffset);
            if(labs(static_cast<long long>(tmNow_ - cur_pos_time)) < 1000){
                lRawTime = cur_pos_time;
            }
            else {
                lRawTime = m_PreRxLocDataSet.m_PositionData.lRecvTime;
            }
        }
        else{
            if(uTimeSpan_ < 0 || uTimeSpan_ > 1000) {
                uTimeSpan_ = 0;
            }
            lRawTime = m_PreRxLocDataSet.m_PositionData.lRecvTime - uTimeSpan_;
        }

        m_aUseInitPos = false;
        return true;
    }
    else {
        return false;
    }
}

bool CRoboLocClnt::SetRoboPose(int nX, int nY, int nTheta, int lX, int lY, int lTheta, int ldX, int ldY, int ldTheta,
                               short uG, unsigned short uN, unsigned long long lPosTime, short nErrCode, short lErrCode,
                               unsigned char chPosMode)
{
    {
        std::lock_guard<std::mutex> lock(data_mtx);

        STPoseData& poseDataTx_ = m_TxLocDataSet.m_PoseData;
        poseDataTx_.nX = nX;
        poseDataTx_.nY = nY;
        poseDataTx_.nTheta = nTheta;
        poseDataTx_.uG = uG;
        poseDataTx_.uN = uN;
        poseDataTx_.lRawTime = lPosTime;  //当前定位数据的时间
        poseDataTx_.chPosMode = chPosMode;
        //poseDataTx_.chOutputMode;

        // 定位失败或通信中断(导航控制器和激光传感器通信中断)
        poseDataTx_.uErrCode = nErrCode;

        poseDataTx_.nLegX = lX;
        poseDataTx_.nLegY = lY;
        poseDataTx_.nLegTheta = lTheta;
        poseDataTx_.nDiffLegX = ldX;
        poseDataTx_.nDiffLegY = ldY;
        poseDataTx_.nDiffLegTheta = ldTheta;
        poseDataTx_.uLegErrCode = lErrCode;
    }
    return true;
}

bool CRoboLocClnt::SetRoboPose(int nX, int nY, int nTheta, short uG, unsigned short uN,
                               unsigned long long lPosTime, short nErrCode, unsigned char chPosMode)
{
    {
        std::lock_guard<std::mutex> lock(data_mtx);

        STPoseData& poseDataTx_ = m_TxLocDataSet.m_PoseData;
        poseDataTx_.nX = nX;
        poseDataTx_.nY = nY;
        poseDataTx_.nTheta = nTheta;
        poseDataTx_.uG = uG;
        poseDataTx_.uN = uN;
        poseDataTx_.lRawTime = lPosTime;  //当前定位数据的时间
        poseDataTx_.chPosMode = chPosMode;
        //poseDataTx_.chOutputMode;

        // 定位失败或通信中断(导航控制器和激光传感器通信中断)
        poseDataTx_.uErrCode = nErrCode;
    }
    return true;
}

// dq VISION 向Cam发送重定位位姿
bool CRoboLocClnt::SetInitPosition_Cam(STPositionData& positionData, int initmode)
{
    std::lock_guard<std::mutex> lock(cam_data_mtx);
    return ClntReportRelocPos_Cam(this, positionData, initmode);
}

// dq VISION 向Cam发送消息暂停定位进程
bool CRoboLocClnt::SetMappingMode_Cam()
{
    return ClntSetMappingMode_Cam(this);
}

int CRoboLocClnt::GetCamMode()
{
    std::lock_guard<std::mutex> lock(data_mtx);
    return m_LocMode_Cam;
}

// dq VISION 发送Cam启动定位模式
bool CRoboLocClnt::SetLocMode_Cam()
{
    return ClntSetLocMode_Cam(this);
}

void CRoboLocClnt::SaveLocPosition_Cam()
{
    std::lock_guard<std::mutex> lock(data_mtx);
    if(m_VecCamData.size() >= 3)
    {
        m_VecCamData.pop_front();
    }
    m_VecCamData.push_back(m_CamLocDataSet);
}

bool CRoboLocClnt::GetLocPosition_Cam(STPositionData& positionData)
{
    std::lock_guard<std::mutex> lock(data_mtx);
    if(m_VecCamData.size() < 1)
        return false;
    positionData = m_VecCamData.back().m_PositionData;
    return true;
}



bool CRoboLocClnt::GetRxLocDataSet(CLocDataSet& data_set)
{
    m_LocCritSect.Lock();
    data_set = m_RxLocDataSet;
    m_LocCritSect.Unlock();
    return true;
}

bool CRoboLocClnt::GetSyncTimeData(STSyncTime& sync_time)
{
    std::lock_guard<std::mutex> lock(sync_time_mtx);

    if(!m_SyncTimeData.bEnable){
        return false;
    }
    sync_time = m_SyncTimeData;

    // 同步时间超时
    unsigned long long tmNow = GetTickCount();
    if(m_SyncTimeData.bRemote){
        long long time_span = static_cast<long long>(tmNow - m_SyncTimeData.lCalibTime);
        if(labs(time_span) < m_SyncTimeData.nTimeOut && m_SyncTimeData.uFlag == 1){
            return true;
        }
        else {
            return false;
        }
    }
    else {
        sync_time.lTimeOffset = 0;
        sync_time.uFlag = 1;
        sync_time.lCalibTime = tmNow;
        return true;
    }
}

int CRoboLocClnt::GetPlsLayer(string ip,unsigned int& key)
{
    if( pslLayerData.size() == 0)
    {
        key = 0;
        return -1;
    }
    for(int i = 0; i < pslLayerData.size();i++)
    {
        STPlsLayer& plsLayerRx_ = pslLayerData[i];
        if(ip == plsLayerRx_.laserIp)
        {
            key = plsLayerRx_.iKey;
            //printf("GetPlsLayer plsLayerRx_ ip %s,%d,i %d\n",plsLayerRx_.laserIp.c_str(),plsLayerRx_.chSetLayer,i);
            return plsLayerRx_.chSetLayer;
        }
    }
    key = 0;
    return -1;
}
void CRoboLocClnt::UpdatePlsState(string ip,int layer,int state,int workstate,unsigned int key)
{
  //  printf("CRoboLocClnt::UpdatePlsState %s, %d\n",ip.c_str(),state);
    std::lock_guard<std::mutex> lock(pls_data_mtx);
    for(int i = 0; i < pslData.size();i++)
    {
        if(ip == pslData[i].laserIp)
        {
            pslData[i].chLayer = layer;
            pslData[i].chState = state;
            pslData[i].chWorkState = workstate;
            pslData[i].iKey = key;
            return;
        }
    }
    STPlsBaseData temp;
    temp.iLaserIpLength = ip.size();
    temp.laserIp = ip;
    temp.chLayer = layer;
    temp.chState = state;
    temp.chWorkState = workstate;
    temp.iKey = key;
    pslData.push_back(temp);
}
