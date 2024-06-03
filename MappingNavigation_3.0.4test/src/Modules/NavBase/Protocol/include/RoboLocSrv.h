//                                     - RoboLocSrv.H -
//
//   定义RoboLocSrv(Robot localization Server)的以太网通讯接口协议。
//

#pragma once

#include <vector>
#include "UdpSrvCom.h"
#include "RoboLocProto.h"
#include "TimeStamp.h"
#include "CPing.h"
#include "Project.h"


//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CRoboLocSrv".
class DllExport CRoboLocSrv : public CUdpServerCom, public CRoboLocProto
{
private:
    bool        m_bStarted;
    bool        m_bUsed;    // 是否启用定位功能
    bool        m_bAutoPos; // Whether to work in auto-mode
    unsigned char m_uCurWorkMode;
    unsigned char m_uNewWorkMode;

public:
    HANDLE     m_hKillThread;       // Handle of "Kill thread" event
    HANDLE     m_hThreadDead;       // Handle of "Thread dead" event
    pthread_t  m_pLocSrvThread;

    HANDLE      m_hKillRecvThread;
    HANDLE      m_hRecvThreadDead;
    pthread_t   m_pLocRecvThread;

    CCriticalSection    m_LocCritSect;

    std::vector<CLocDataSet*> m_RxLocDataSet;
    std::vector<CLocDataSet*> m_TxLocDataSet;

    int     m_nClntCount;
    std::string m_strVersion;

    CPing m_Ping;

public:
    // Default constructor
    CRoboLocSrv(USHORT uRemotePort, BOOL bFramed, BOOL bAscii);

    ~CRoboLocSrv();

    bool Initialize();

    // Install the RLP protocol
    BOOL Install();

    void Reset();

    BOOL Create();

    BOOL Stop();

    bool IsUsed() {return m_bUsed;}

    // The support routine for RLP protocol
    void SupportRoutine();

    void RLRunStage();

    void ProcessReceiveData();

    void ProcessNetStatus();

    void ProcessDataSend();

    // Start/stop the auto-positioning agent
    void StartAutoPosAgent(bool bYes = true);

    // Support auto-positioning
    void SupportAutoPosAgent();

    // 设置新的工作模式
    void SetNewWorkMode(unsigned char uMode) {m_uNewWorkMode = uMode;}

private:
    // 切换定位系统工作模式
    bool ChangeMode();

    // 空闲
    bool LocStandBy();

    // 执行建图过程
    bool DoMapping();

    // 执行定位过程
    bool DoLocalization();

    // 执行扫描过程
    bool DoScan();

    // 获取建图数据
    bool AutoGetMap();

    // 获取机器人位姿数据
    bool AutoGetPose(int nChannel);

    // 获取扫描数据
    bool AutoGetScan();

    // 识别位姿是否有效
    bool RecognisePose();

private:
    // 解析客户端反馈的数据.
    bool ClntReportLocInfoDe(int nChannel);

    bool ClntReplyAttrDe(int nChannel);

    bool ClntReportAttrDe(int nChannel);

    bool ClntReportSyncTimeDe(int nChannel);

    bool ClntGetSyncTimeDe(int nChannel);

    bool ClntReplyModeDe(int nChannel);

    bool ClntReportModeDe(int nChannel);

    bool ClntReplyPositionDe(int nChannel);

    bool ClntReportPoseDe(int nChannel);

    bool ClntReportHeartBeatDe(int nChannel);

    // 发送服务器端的数据.
    bool SrvGetLocInfoSe(int nChannel, STLocInfo& locInfo);

    bool SrvSetAttrSe(int nChannel, STLocAttr& locAttr);

    bool SrvGetAttrSe(int nChannel, STLocAttr& locAttr);

    bool SrvGetSyncTimeSe(int nChannel, STSyncTime& syncTime);

    bool SrvReportSyncTimeSe(int nChannel, STSyncTime& syncTime);

    bool SrvSetModeSe(int nChannel, STOperateMode& operateMode);

    bool SrvGetModeSe(int nChannel, STOperateMode& operateMode);

    bool SrvSetPositionSe(int nChannel, STPositionData& positionData);

    bool SrvGetPoseSe(int nChannel, STVelocityData& velocityData);

    bool SrvSendHeartBeatSe(int nChannel, STHeartBeat& heartBeat);

private:
    // 接收到客户端的应答
    bool OnClientEcho(int nChannel);

    // PING
    int Ping(char* strHost);

    // 异步处理读取定位系统信息数据帧
    bool HandleLocInfoFrame(int nChannel);

    // 异步处理设置定位系统属性数据帧
    bool HandleLocAttrFrame(int nChannel);

    // 异步处理设置定位系统模式数据帧
    bool HandleLocModeFrame();

    // 异步处理设置机器人初始位姿数据帧
    bool HandleRoboPositionFrame();

    // 异步处理获取机器人位姿数据帧
    bool HandleRoboPoseFrame();

public:
    // 网络通讯是否堵塞.
    bool IsBlocked(int nChannel);

    // 获取定位系统ID和软件版本号.
    bool GetLocInfo(unsigned char& chId, unsigned int& nSwVersion);

    // 设置定位系统属性，作用半径、应用最近特征数.
    bool SetLocAttr(unsigned int nRadiusFrom, unsigned int nRadiusTo, unsigned int NClosest);

    // 获取定位系统属性，作用半径、应用最近特征数.
    bool GetLocAttr(unsigned int& nRadiusFrom, unsigned int& nRadiusTo, unsigned int& NClosest);

    // 设置定位系统工作模式
    bool SetLocMode(unsigned char chMode);

    // 获取定位系统工作模式
    bool GetLocMode(unsigned char& chMode);

    // 设置定位系统当前位姿
    bool SetRoboPosition(unsigned char chMapId, int nX, int nY, int nTheta, unsigned long long lTime);

    // 获取定位系统位姿
    bool GetRoboPose(short Vx, short Vy, short Vtheta, unsigned long long lTime,
                    unsigned char coordBase, unsigned char Wait, unsigned char Mask);
};

