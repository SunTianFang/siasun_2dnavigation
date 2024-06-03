//                                - CRoboLocSrv.CPP -
//
//   Implementatin of class "CRoboLocSrv".
//

#include "stdafx.h"
#include "RoboLocSrv.h"
#include "UdpChannel.h"
#include <fstream>
#include "include/json/json.h"
#include "Tools.h"
#include "blackboxhelper.hpp"
#include "StdAgv.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CStdAGV      AGV;


#if defined USE_BLACK_BOX
extern CBlackBox LaserBox;
#endif


//////////////////////////////////////////////////////////////////////////////
//   The support routine of Robot localization protocol.
void* RLocSrvSupportProc(LPVOID pParam)
{
    CRoboLocSrv* pRLocSrv = reinterpret_cast<CRoboLocSrv*>(pParam);
    while (WaitForSingleObject(pRLocSrv->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
        pRLocSrv->m_LocCritSect.Lock();
        pRLocSrv->SupportRoutine();
        pRLocSrv->m_LocCritSect.Unlock();

        Sleep(RLS_CTRL_CYCLE);
    }

    SetEvent(pRLocSrv->m_hThreadDead);
    pthread_exit(NULL);

    return NULL;
}

//
//   The thread procedure for the deferred call.
//
void* RLocSrvRecvThreadProc(LPVOID pParam)
{
    CRoboLocSrv* pRLocSrv = reinterpret_cast<CRoboLocSrv*>(pParam);

    // Loop until the kill event is set
    while (WaitForSingleObject(pRLocSrv->m_hKillRecvThread, 0) != WAIT_OBJECT_0)
    {
        pRLocSrv->m_LocCritSect.Lock();
        pRLocSrv->ProcessReceiveData();
        pRLocSrv->m_LocCritSect.Unlock();

        // Waiting for calling event if this is a "hard" timer
        sem_wait(pRLocSrv->m_hRecvCalling); // Block Wait
    }

    SetEvent(pRLocSrv->m_hRecvThreadDead);
    pthread_exit(NULL);

    return NULL;
}

///////////////////////////////////////////////////////////////////////////////

CRoboLocSrv::CRoboLocSrv(USHORT uRemotePort, BOOL bFramed, BOOL bAscii) :
    CUdpServerCom(uRemotePort, bFramed, bAscii)
{
    m_bStarted = false;
    m_bUsed = false;

    m_hKillThread = NULL;
    m_hThreadDead = NULL;
    m_pLocSrvThread = 0;

    m_hKillRecvThread = NULL;
    m_hRecvThreadDead = NULL;
    m_pLocRecvThread = 0;

    m_hRecvCalling = NULL;
    m_nClntCount = 0;
    m_strVersion = "1.0";

    m_RxLocDataSet.clear();
    m_TxLocDataSet.clear();

    m_uCurWorkMode = RLP_MODE_STANDBY;
    m_uNewWorkMode = RLP_MODE_STANDBY;

    Reset();
}

CRoboLocSrv::~CRoboLocSrv()
{
    Stop();

    for (unsigned int i = 0; i < m_RxLocDataSet.size(); i++)
    {
        if (m_RxLocDataSet[i])
        {
            delete m_RxLocDataSet[i];
            m_RxLocDataSet[i] = NULL;
        }
    }
    m_RxLocDataSet.clear();

    for (unsigned int i = 0; i < m_TxLocDataSet.size(); i++)
    {
        if (m_TxLocDataSet[i])
        {
            delete m_TxLocDataSet[i];
            m_TxLocDataSet[i] = NULL;
        }
    }
    m_TxLocDataSet.clear();
}

//
bool CRoboLocSrv::Initialize()
{
    bool bRet = false;

    if(!Create())
    {
        return false;
    }

    bRet = Install();

    return bRet;
}

//
//   Install the RLP manager.
//
BOOL CRoboLocSrv::Install()
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
    if(pthread_create(&m_pLocSrvThread, &attr1, RLocSrvSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RLocSrvSupportProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
    {
        std::cout<<"Creat RLocSrvSupportProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr1);


    m_hKillRecvThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    m_hRecvThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    m_hRecvCalling = CreateEvent(NULL, FALSE, FALSE, NULL);

    if (m_hKillRecvThread == NULL || m_hRecvThreadDead == NULL || m_hRecvCalling == NULL)
        return FALSE;

    pthread_attr_t attr2;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr2);
    if(pthread_create(&m_pLocRecvThread, &attr2, RLocSrvRecvThreadProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RLocSrvRecvThreadProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
    {
        std::cout<<"Creat RLocSrvRecvThreadProc Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr2);

    Reset();

    m_bStarted = true;

    return TRUE;
}

void CRoboLocSrv::Reset()
{

}

//
BOOL CRoboLocSrv::Create()
{	
    BOOL bRet = FALSE;

    std::ifstream FileLocParm(WORK_PATH"RoboLocParm.json");
    Json::Reader Jreader;
    Json::Value LocParmRoot;

    if(!FileLocParm)
    {
        return FALSE;
    }

    if (Jreader.parse(FileLocParm, LocParmRoot))
    {
        // Server parameter
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

        // Client parameter
        if (!LocParmRoot["LocClient"].isNull())
        {
            m_nClntCount = LocParmRoot["LocClient"].size();
        }
        else
        {
            m_nClntCount = 0;
        }

        if (m_nClntCount > MAX_UDP_CLIENTS_COUNT)
            m_nClntCount = MAX_UDP_CLIENTS_COUNT;

        for (int i = 0; i < m_nClntCount; i++)
        {
            int nId = 0;
            char uchIpBuf[16];
            memset(uchIpBuf, 0, 16 * sizeof(char));

            if (!LocParmRoot["LocClient"][i]["ID"].isNull())
            {
                nId = LocParmRoot["LocClient"][i]["ID"].asInt();
            }

            if (!LocParmRoot["LocClient"][i]["IP"].isNull())
            {
                unsigned long nCount = LocParmRoot["LocClient"][i]["IP"].asString().size();
                if (nCount < 16)
                {
                    memcpy(uchIpBuf, LocParmRoot["LocClient"][i]["IP"].asString().c_str(), nCount * sizeof(char));
                }
            }

            CreateCom(uchIpBuf);

            CLocDataSet* pRxDataSet = new CLocDataSet();
            pRxDataSet->Initialize();
            m_RxLocDataSet.push_back(pRxDataSet);

            CLocDataSet* pTxDataSet = new CLocDataSet();
            pTxDataSet->Initialize();
            m_TxLocDataSet.push_back(pTxDataSet);
        }

        bRet = TRUE;
    }

    FileLocParm.close();

    return bRet;
}

BOOL CRoboLocSrv::Stop()
{
    if (!m_bStarted)
        return FALSE;

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
    PthreadJoin(m_pLocSrvThread);

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
//   The support routine of the network manager.
//
void CRoboLocSrv::SupportRoutine()
{
    SupportAutoPosAgent();

    RLRunStage();

    ProcessNetStatus();

    ProcessDataSend();
}

//
void CRoboLocSrv::RLRunStage()
{
    int i = 0;

    for (i = 0; i < m_nClntCount; i++)
    {
        BOOL bChannelExist = SelectChannel(i);
        if (!bChannelExist)
        {
            continue;
        }

        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
        {
            continue;
        }

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        switch(m_TxLocDataSet[n_]->m_uLocState)
        {
        case RLP_MODE_POWERON:
            break;

        case RLP_MODE_STANDBY:
            break;

        case RLP_MODE_MAPPING:
            break;

        case RLP_MODE_LOCALIZATION:
            break;

        case RLP_MODE_SCAN:
            break;

        default:
            break;
        }
    }
}

//
void CRoboLocSrv::ProcessReceiveData()
{
    char chDummy;
    USHORT uType;
    int i;

    for (i = 0; i < m_nClntCount; i++)
    {
        BOOL bChannelExist = SelectChannel(i);
        if (!bChannelExist)
        {
            continue;
        }

        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
            continue;

        while(RxReady(i))
        {
            // 对数据进行预处理
            if (!pChannel->Preprocess())
                break;

            if (!pChannel->GetRxHeader())
                continue;

            *pChannel >> uType;

            switch(uType)
            {
            case RLP_CLNT_REPORT_LOCINFO:
                ClntReportLocInfoDe(i);
                break;

            case RLP_CLNT_REPLY_ATTR:
                ClntReplyAttrDe(i);
                break;

            case RLP_CLNT_REPORT_ATTR:
                ClntReportAttrDe(i);
                break;

            case RLP_CLNT_REPORT_SYNC_TIME:
                ClntReportSyncTimeDe(i);
                break;

            case RLP_CLNT_GET_SYNC_TIME:
                ClntGetSyncTimeDe(i);
                break;

            case RLP_CLNT_REPLY_MODE:
                ClntReplyModeDe(i);
                break;

            case RLP_CLNT_REPORT_MODE:
                ClntReportModeDe(i);
                break;

            case RLP_CLNT_REPLY_POSITION:
                ClntReplyPositionDe(i);
                break;

            case RLP_CLNT_REPORT_POSE:
            {
                bool bRet = ClntReportPoseDe(i);
                // 设置机器人位姿
                if(bRet)
                {
                    AutoGetPose(i);
                    OnClientEcho(i);
                }
            }
                break;

            case RLP_CLNT_REPORT_HEARTBEAT:
            {
                bool bRet = ClntReportHeartBeatDe(i);
                if(bRet)
                {
                    unsigned long long tmNow = GetTickCount();
                    unsigned long n_ = static_cast<unsigned long>(i);
                    m_RxLocDataSet[n_]->m_HeartBeat.lRecvTime = tmNow;
                    if(tmNow - m_RxLocDataSet[n_]->m_HeartBeat.lSrvTime < RLP_NET_RECV_TIMEOUT)
                    {
                        OnClientEcho(i);
                    }
                }
            }
                break;

            default:
                FILE_BlackBox(LaserBox, "--> Slave sends error command: ", uType);
                uType = RLP_CLNT_COMMAND_ERROR;

                if(!pChannel->IsAscii())
                {
                    while (RxReady(i))
                        (*pChannel->GetChannelObject()) >> chDummy;
                }

                break;
            }
        }
    }
}

// 判断网络通讯状态
void CRoboLocSrv::ProcessNetStatus()
{
    unsigned long long tmNow = GetTickCount();     // Get the current time
    int i = 0;

    for (i = 0; i < m_nClntCount; i++)
    {
        bool bChannelExist = SelectChannel(i);
        if (!bChannelExist)
        {
            continue;
        }

        CUdpChannel* pChannel;
        pChannel = GetChannel(i);

        if (pChannel == NULL || !pChannel->InUse())
        {
            continue;
        }

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size())
        {
            continue;
        }

        /// 数据交互超时,认为通讯中断
        if (!m_RxLocDataSet[n_]->m_bNetBlocked && (tmNow - m_RxLocDataSet[n_]->m_tLastEcho > RLP_NET_COMM_TIMEOUT))
        {
            m_RxLocDataSet[n_]->m_bNetBlocked = true;

            FILE_BlackBox(LaserBox, "--> Slave Network blocked: i = ", i);
        }

        if (m_RxLocDataSet[n_]->m_bNetBlocked)
        {
            // 发送PING指令
        }

        // 发送心跳数据
        STHeartBeat& heartBeatTx_ = m_TxLocDataSet[n_]->m_HeartBeat;
        STHeartBeat& heartBeatRx_ = m_RxLocDataSet[n_]->m_HeartBeat;

        heartBeatTx_.lSrvTime = GetTickCount();
        if(heartBeatRx_.lRecvTime > 0)
        {
            heartBeatTx_.lClntTime = heartBeatTx_.lClntTime + tmNow - heartBeatRx_.lRecvTime;
        }

        if(tmNow - m_TxLocDataSet[n_]->m_tLastSend > RLP_NET_REPORT_CYCLE)
        {
            SrvSendHeartBeatSe(i, heartBeatTx_);
            m_TxLocDataSet[n_]->m_tLastSend = tmNow;
        }
    }
}

//
void CRoboLocSrv::ProcessDataSend()
{
    int i = 0;

    for (i = 0; i < m_nClntCount; i++)
    {
        BOOL bChannelExist = SelectChannel(i);
        if (!bChannelExist)
        {
            continue;
        }

        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
            continue;

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        if (m_RxLocDataSet[n_]->m_bNetBlocked)
        {
            continue;
        }

        // 发送查询定位系统信息
        HandleLocInfoFrame(i);

        // 发送定位系统属性数据
        HandleLocAttrFrame(i);
    }
}

// Start/stop the auto-positioning agent
void CRoboLocSrv::StartAutoPosAgent(bool bYes)
{
    m_bAutoPos = bYes;
}

// Support auto-positioning
void CRoboLocSrv::SupportAutoPosAgent()
{
    // 根据选择设置工作模式
    if(AGV.m_bMappingModel)
    {
        SetNewWorkMode(RLP_MODE_MAPPING);
    }
    else
    {
        SetNewWorkMode(RLP_MODE_LOCALIZATION);
    }

    // 如果定位数据超时认为无效
    RecognisePose();

    ChangeMode();

    switch(m_uCurWorkMode)
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

bool CRoboLocSrv::ChangeMode()
{
    bool bRet = false;
    unsigned char uMode_;

    if(m_uCurWorkMode != m_uNewWorkMode)
    {
        // 更改成新模式
        SetLocMode(m_uNewWorkMode);

        bRet = HandleLocModeFrame();
        if(bRet && GetLocMode(uMode_) && uMode_ == m_uNewWorkMode)
        {
            m_uCurWorkMode = m_uNewWorkMode;
            bRet = true;
        }
    }

    return bRet;
}

bool CRoboLocSrv::LocStandBy()
{
    return true;
}

bool CRoboLocSrv::DoMapping()
{
    return true;
}

bool CRoboLocSrv::DoLocalization()
{
    // 设置定位系统初始位姿
    CPosture stNewPosture;
    if (AGV.LaserNavSys.GetNewPosture(stNewPosture))
    {
        unsigned char chMapId_ = 0;     // 默认为0
        unsigned long long lTime_ = GetTickCount();
        int nX_ = static_cast<int>(stNewPosture.x * 1000);
        int nY_ = static_cast<int>(stNewPosture.y * 1000);
        int nTheta_ = static_cast<int>(RAD_TO_BDEG(stNewPosture.fThita) * 1000);

        SetRoboPosition(chMapId_, nX_, nY_, nTheta_, lTime_);
    }
    else
    {
        // 设置定位系统速度矢量
        CVelocity EstVel;
        float vel_x = 0, vel_y = 0, vel_angle = 0;
        CFloatDataInfo GyroAngle ;

        AGV.LaserNavSys.m_AutoPosData.GetLocalVel(vel_x, vel_y, vel_angle);

        // 使用陀螺仪的角度
        if(fabs(vel_x) > 0 || fabs(vel_y) > 0)
        {
            AGV.LaserNavSys.m_bStartGyro = true;
        }

        if(AGV.LaserNavSys.m_bStartGyro == true)
        {
            AGV.GyroNavSys.GetRelAngle(GyroAngle);
        }

        if(isnan(vel_x) || isnan(vel_y) || isnan(vel_angle))
        {
            vel_x = 0;
            vel_y = 0;
            vel_angle = 0;
        }

        short Vx_ = static_cast<short>(vel_x * 1000);
        short Vy_ = static_cast<short>(vel_y * 1000);
        short Vtheta_ = static_cast<short>(vel_angle * 1000);
        unsigned long long lTime_ = GetTickCount(); // 应该是输入速度矢量时的当前时间,而不是从此处取时间
        unsigned char coordBase_ = RLP_LOCAL_COORDINATE;
        unsigned char Wait_ = RLP_INSTANTLY_LAST_POSE;
        unsigned char Mask_ = RLP_ONLY_POSE;

        GetRoboPose(Vx_, Vy_, Vtheta_, lTime_, coordBase_, Wait_, Mask_);
    }

    // 发送定位系统当前位姿数据
    HandleRoboPositionFrame();

    // 发送定位系统当前速度矢量
    HandleRoboPoseFrame();

    return true;
}

bool CRoboLocSrv::DoScan()
{
    return true;
}

bool CRoboLocSrv::AutoGetMap()
{
    return true;
}

bool CRoboLocSrv::AutoGetPose(int nChannel)
{
    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(n_ >= m_RxLocDataSet.size())
    {
        return false;
    }

    STPoseData& poseData_ = m_RxLocDataSet[n_]->m_PoseData;

    float fX_ = poseData_.nX / 1000;
    float fY_ = poseData_.nY / 1000;
    float fThita_ = BDEG_TO_RAD(poseData_.nTheta / 1000);
    unsigned char uchG_ = static_cast<unsigned char>(poseData_.uG);
    unsigned char uchN_ = static_cast<unsigned char>(poseData_.uN);

    // 接收到定位数据的时间，使用定位数据时可以根据时间进行插补;如果定位数据超过一定时间，认为无效
    unsigned long long lRecvTime_ = poseData_.lRecvTime;

    AGV.LaserNavSys.m_AutoPosData.SetPos(fX_, fY_, fThita_, uchG_, uchN_);

    return true;
}

bool CRoboLocSrv::AutoGetScan()
{
    return true;
}

// 识别位姿是否有效
bool CRoboLocSrv::RecognisePose()
{
    int i = 0;

    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size())
        {
            continue;
        }

        STPoseData& poseData_ = m_RxLocDataSet[n_]->m_PoseData;

        // 接收到定位数据的时间，如果定位数据超过一定时间，认为无效
        unsigned long long lNow_ = GetTickCount();
        unsigned long long lRecvTime_ = poseData_.lRecvTime;

        if(lNow_ - lRecvTime_ > RLP_GET_POSE_TIMEOUT)
        {
            float fX_ = 0;
            float fY_ = 0;
            float fThita_ = 0;
            unsigned char uchG_ = 0;
            unsigned char uchN_ = 0;

            AGV.LaserNavSys.m_AutoPosData.SetPos(fX_, fY_, fThita_, uchG_, uchN_);
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////
bool CRoboLocSrv::ClntReportLocInfoDe(int nChannel)
{
    bool bResult = false;
    STLocInfo locInfo_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportLocInfoDe(pChannel, locInfo_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_LocInfo = locInfo_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReplyAttrDe(int nChannel)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReplyAttrDe(pChannel, locAttr_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_LocAttr = locAttr_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReportAttrDe(int nChannel)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportAttrDe(pChannel, locAttr_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_LocAttr = locAttr_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReportSyncTimeDe(int nChannel)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportSyncTimeDe(pChannel, syncTime_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_SyncTime = syncTime_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntGetSyncTimeDe(int nChannel)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntGetSyncTimeDe(pChannel, syncTime_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_SyncTime = syncTime_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReplyModeDe(int nChannel)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReplyModeDe(pChannel, operateMode_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_OperateMode = operateMode_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReportModeDe(int nChannel)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportModeDe(pChannel, operateMode_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_OperateMode = operateMode_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReplyPositionDe(int nChannel)
{
    bool bResult = false;
    STPositionData positionData_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReplyPositionDe(pChannel, positionData_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_PositionData = positionData_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReportPoseDe(int nChannel)
{
    bool bResult = false;
    STPoseData poseData_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportPoseDe(pChannel, poseData_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_PoseData = poseData_;
    }

    return bResult;
}

///
bool CRoboLocSrv::ClntReportHeartBeatDe(int nChannel)
{
    bool bResult = false;
    STHeartBeat heartBeat_;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::ClntReportHeartBeatDe(pChannel, heartBeat_);

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(bResult && n_ < m_RxLocDataSet.size())
    {
        m_RxLocDataSet[n_]->m_HeartBeat = heartBeat_;
    }

    return bResult;
}

///
bool CRoboLocSrv::SrvGetLocInfoSe(int nChannel, STLocInfo& locInfo)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvGetLocInfoSe(pChannel, locInfo);

    return bResult;
}

///
bool CRoboLocSrv::SrvSetAttrSe(int nChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvSetAttrSe(pChannel, locAttr);

    return bResult;
}

///
bool CRoboLocSrv::SrvGetAttrSe(int nChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvGetAttrSe(pChannel, locAttr);

    return bResult;
}

///
bool CRoboLocSrv::SrvGetSyncTimeSe(int nChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvGetSyncTimeSe(pChannel, syncTime);

    return bResult;
}

///
bool CRoboLocSrv::SrvReportSyncTimeSe(int nChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvReportSyncTimeSe(pChannel, syncTime);

    return bResult;
}

///
bool CRoboLocSrv::SrvSetModeSe(int nChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvSetModeSe(pChannel, operateMode);

    return bResult;
}

///
bool CRoboLocSrv::SrvGetModeSe(int nChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvGetModeSe(pChannel, operateMode);

    return bResult;
}

///
bool CRoboLocSrv::SrvSetPositionSe(int nChannel, STPositionData& positionData)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvSetPositionSe(pChannel, positionData);

    return bResult;
}

///
bool CRoboLocSrv::SrvGetPoseSe(int nChannel, STVelocityData& velocityData)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvGetPoseSe(pChannel, velocityData);

    return bResult;
}

///
bool CRoboLocSrv::SrvSendHeartBeatSe(int nChannel, STHeartBeat& heartBeat)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    CUdpChannel* pChannel = GetChannel(nChannel);
    bResult = CRoboLocProto::SrvSendHeartBeatSe(pChannel, heartBeat);

    return bResult;
}

//
// The client Echo
//
bool CRoboLocSrv::OnClientEcho(int nChannel)
{
    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(n_ >= m_RxLocDataSet.size())
    {
        return false;
    }
    m_RxLocDataSet[n_]->m_tLastEcho = GetTickCount();
    m_RxLocDataSet[n_]->m_bNetBlocked = false;

    return true;
}

//
int CRoboLocSrv::Ping(char* strHost)
{
    return m_Ping.Ping(strHost);
}

///
bool CRoboLocSrv::HandleLocInfoFrame(int nChannel)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
    {
        return false;
    }

    STLocInfo& locInfoRx_ = m_RxLocDataSet[n_]->m_LocInfo;
    STLocInfo& locInfoTx_ = m_TxLocDataSet[n_]->m_LocInfo;
    if (!locInfoRx_.bUpdate)
    {
       SrvGetLocInfoSe(nChannel, locInfoTx_);
    }
    else
    {
        bResult = true;
    }

    return bResult;
}

///
bool CRoboLocSrv::HandleLocAttrFrame(int nChannel)
{
    bool bResult = false;

    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
    {
        return false;
    }

    STLocAttr& locAttrRx_ = m_RxLocDataSet[n_]->m_LocAttr;
    STLocAttr& locAttrTx_ = m_TxLocDataSet[n_]->m_LocAttr;

    if(locAttrTx_.bUpdate && (!locAttrRx_.bUpdate || locAttrRx_.nRadiusFrom != locAttrTx_.nRadiusFrom
            || locAttrRx_.nRadiusTo != locAttrTx_.nRadiusTo || locAttrRx_.NClosest != locAttrTx_.NClosest))
    {
        SrvSetAttrSe(nChannel, locAttrTx_);
    }
    else
    {
        locAttrTx_.bUpdate = false;
        bResult = true;
    }

    return bResult;
}

///
bool CRoboLocSrv::HandleLocModeFrame()
{
    int i = 0;
    bool bResult = false;

    for (i = 0; i < m_nClntCount; i++)
    {
        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
            continue;

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        if (m_RxLocDataSet[n_]->m_bNetBlocked)
        {
            continue;
        }

        STOperateMode& opModeRx_ = m_RxLocDataSet[n_]->m_OperateMode;
        STOperateMode& opModeTx_ = m_TxLocDataSet[n_]->m_OperateMode;

        if(opModeTx_.bUpdate && (!opModeRx_.bUpdate || opModeRx_.chMode != opModeTx_.chMode))
        {
            SrvSetModeSe(i, opModeTx_);
        }
        else
        {
            opModeTx_.bUpdate = false;
            bResult = true;
        }
    }

    return bResult;
}

///
bool CRoboLocSrv::HandleRoboPositionFrame()
{
    int i = 0;
    bool bResult = false;

    for (i = 0; i < m_nClntCount; i++)
    {
        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
            continue;

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        if (m_RxLocDataSet[n_]->m_bNetBlocked)
        {
            continue;
        }

        STPositionData& posDataRx_ = m_RxLocDataSet[n_]->m_PositionData;
        STPositionData& posDataTx_ = m_TxLocDataSet[n_]->m_PositionData;

        if(posDataTx_.bUpdate && (!posDataRx_.bUpdate || posDataRx_.chMapId != posDataTx_.chMapId
                                  || posDataRx_.nX != posDataTx_.nX || posDataRx_.nY != posDataTx_.nY
                                  || posDataRx_.nTheta != posDataTx_.nTheta))
        {
            SrvSetPositionSe(i, posDataTx_);
        }
        else
        {
            posDataTx_.bUpdate = false;
            bResult = true;
        }
    }

    return bResult;
}

///
bool CRoboLocSrv::HandleRoboPoseFrame()
{
    int i = 0;
    bool bResult = false;

    for (i = 0; i < m_nClntCount; i++)
    {
        CUdpChannel* pChannel;
        pChannel = GetChannel(i);
        if (pChannel == NULL || !pChannel->InUse())
            continue;

        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        if (m_RxLocDataSet[n_]->m_bNetBlocked)
        {
            continue;
        }

        STVelocityData& velDataRx_ = m_RxLocDataSet[n_]->m_VelocityData;
        STVelocityData& velDataTx_ = m_TxLocDataSet[n_]->m_VelocityData;

        if(velDataTx_.bUpdate && (!velDataRx_.bUpdate || velDataRx_.Vx != velDataTx_.Vx || velDataRx_.Vy != velDataTx_.Vy
                                  || velDataRx_.Vtheta != velDataTx_.Vtheta || velDataRx_.lTime != velDataTx_.lTime
                                  || velDataRx_.coordBase != velDataTx_.coordBase || velDataRx_.Wait != velDataTx_.Wait
                                  || velDataRx_.Mask != velDataTx_.Mask))
        {
            SrvGetPoseSe(i, velDataTx_);
        }
        else
        {
            velDataTx_.bUpdate = false;
            bResult = true;
        }
    }

    return bResult;
}

////////////////////////////////////////////////////////////////////
// 网络通讯是否堵塞.
bool CRoboLocSrv::IsBlocked(int nChannel)
{
    if (nChannel < 0 || nChannel > m_nClntCount)
        return false;

    unsigned long n_ = static_cast<unsigned long>(nChannel);
    if(n_ >= m_RxLocDataSet.size())
    {
        return false;
    }

    return m_RxLocDataSet[n_]->m_bNetBlocked;
}

// 获取设备ID和软件版本号.
bool CRoboLocSrv::GetLocInfo(unsigned char& chDevId, unsigned int& nSwVersion)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STLocInfo& locInfoRx_ = m_RxLocDataSet[n_]->m_LocInfo;
        STLocInfo locInfoTx_;
        if (locInfoRx_.bUpdate)
        {
            chDevId = locInfoRx_.ID;
            nSwVersion = locInfoRx_.nSwVersion;
            bRet = true;
        }
        else
        {
            SrvGetLocInfoSe(i, locInfoTx_);
        }
    }

    return bRet;
}

// 设置设备属性，作用半径、应用最近特征数.
bool CRoboLocSrv::SetLocAttr(unsigned int nRadiusFrom, unsigned int nRadiusTo, unsigned int NClosest)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STLocAttr& locAttrRx_ = m_RxLocDataSet[n_]->m_LocAttr;
        STLocAttr& locAttrTx_ = m_TxLocDataSet[n_]->m_LocAttr;

        if(locAttrTx_.nRadiusFrom != nRadiusFrom || locAttrTx_.nRadiusTo != nRadiusTo
                || locAttrTx_.NClosest != NClosest)
        {
            locAttrTx_.nRadiusFrom = nRadiusFrom;
            locAttrTx_.nRadiusTo = nRadiusTo;
            locAttrTx_.NClosest = NClosest;
            locAttrTx_.lTime = GetTickCount();
            locAttrTx_.bUpdate = true;
            locAttrRx_.bUpdate = false;
        }

        bRet = true;
    }

    return bRet;
}

// 获取设备属性，作用半径、应用最近特征数.
bool CRoboLocSrv::GetLocAttr(unsigned int& nRadiusFrom, unsigned int& nRadiusTo, unsigned int& NClosest)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STLocAttr& locAttrRx_ = m_RxLocDataSet[n_]->m_LocAttr;
        STLocAttr& locAttrTx_ = m_TxLocDataSet[n_]->m_LocAttr;

        if(locAttrRx_.bUpdate)
        {
            nRadiusFrom = locAttrRx_.nRadiusFrom;
            nRadiusTo = locAttrRx_.nRadiusTo;
            NClosest = locAttrRx_.NClosest;

            bRet = true;
        }
        else
        {
            SrvGetAttrSe(i, locAttrTx_);
        }
    }

    return bRet;
}

// 设置设备工作模式
bool CRoboLocSrv::SetLocMode(unsigned char chMode)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STOperateMode& opModeRx_ = m_RxLocDataSet[n_]->m_OperateMode;
        STOperateMode& opModeTx_ = m_TxLocDataSet[n_]->m_OperateMode;
        if(opModeTx_.chMode != chMode)
        {
            opModeTx_.chMode = chMode;
            opModeTx_.bUpdate = true;
            opModeRx_.bUpdate = false;
        }

        bRet = true;
    }

    return bRet;
}

// 获取设备工作模式
bool CRoboLocSrv::GetLocMode(unsigned char& chMode)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STOperateMode& operateModeRx_ = m_RxLocDataSet[n_]->m_OperateMode;
        STOperateMode& operateModeTx_ = m_TxLocDataSet[n_]->m_OperateMode;

        if(operateModeRx_.bUpdate)
        {
            chMode = operateModeRx_.chMode;
            bRet = true;
        }
        else
        {
            SrvGetModeSe(i, operateModeTx_);
        }
    }

    return bRet;
}

// 设置设备当前位姿
bool CRoboLocSrv::SetRoboPosition(unsigned char chMapId, int nX, int nY, int nTheta, unsigned long long lTime)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STPositionData& posDataRx_ = m_RxLocDataSet[n_]->m_PositionData;
        STPositionData& posDataTx_ = m_TxLocDataSet[n_]->m_PositionData;
        if(posDataTx_.chMapId != chMapId || posDataTx_.nX != nX || posDataTx_.nY != nY
                || posDataTx_.nTheta != nTheta /*|| posDataTx_.lTime != lTime*/)
        {
            posDataTx_.chMapId = chMapId;
            posDataTx_.nX = nX;
            posDataTx_.nY = nY;
            posDataTx_.nTheta = nTheta;
            posDataTx_.lTime = lTime;
            posDataTx_.bUpdate = true;
            posDataRx_.bUpdate = false;
        }

        bRet = true;
    }

    return bRet;
}

// 获取当前设备位姿
bool CRoboLocSrv::GetRoboPose(short Vx, short Vy, short Vtheta, unsigned long long lTime,
                             unsigned char coordBase, unsigned char Wait, unsigned char Mask)
{
    bool bRet = false;
    int i = 0;

    // 目前只有一个客户端
    for (i = 0; i < m_nClntCount; i++)
    {
        unsigned long n_ = static_cast<unsigned long>(i);
        if(n_ >= m_RxLocDataSet.size() || n_ >= m_TxLocDataSet.size())
        {
            continue;
        }

        STVelocityData& velDataRx_ = m_RxLocDataSet[n_]->m_VelocityData;
        STVelocityData& velDataTx_ = m_TxLocDataSet[n_]->m_VelocityData;
        if(velDataTx_.Vx != Vx || velDataTx_.Vy != Vy || velDataTx_.Vtheta != Vtheta || velDataTx_.lTime != lTime
                || velDataTx_.coordBase != coordBase || velDataTx_.Wait != Wait || velDataTx_.Mask != Mask)
        {
            velDataTx_.Vx = Vx;
            velDataTx_.Vy = Vy;
            velDataTx_.Vtheta = Vtheta;
            velDataTx_.lTime = lTime;
            velDataTx_.coordBase = coordBase;
            velDataTx_.Wait = Wait;
            velDataTx_.Mask = Mask;
            velDataTx_.bUpdate = true;
            velDataRx_.bUpdate = false;
        }

        bRet = true;
    }

    return bRet;
}

