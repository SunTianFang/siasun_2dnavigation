//                                     - RoboLocClnt.H -
//
//   定义RoboLocClnt(Robot localization Client)的以太网通讯接口协议.
//

#pragma once

#include <mutex>
#include <atomic>
#include "UdpChannel.h"
#include "RoboLocProto.h"
#include "MagicSingleton.h"
#include "TimeStamp.h"
#include "Project.h"

class CAgvUdpSocket;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CRoboLocClnt".
class CRoboLocClnt : public CUdpChannel, public CRoboLocProto
{
private:
    bool        m_bStarted;
    bool        m_bUsed;    // 是否启用定位功能
    std::atomic_uchar m_aCurWorkMode;
    std::atomic_uchar m_aNewWorkMode;
    atomic_bool   m_aUseInitPos;
    std::mutex  data_mtx;
    std::mutex  pls_data_mtx;
    std::mutex  cam_data_mtx;
    STSyncTime  m_SyncTimeData;
    std::mutex  sync_time_mtx;
    std::vector<STPlsBaseData> pslData;
    std::vector<STPlsLayer> pslLayerData;
    bool        m_bUseSoftPls;

public:
    HANDLE     m_hKillThread;       // Handle of "Kill thread" event
    HANDLE     m_hThreadDead;       // Handle of "Thread dead" event
    pthread_t  m_pLocClntThread;

    HANDLE      m_hKillRecvThread;
    HANDLE      m_hRecvThreadDead;
    pthread_t   m_pLocRecvThread;

    CAgvUdpSocket*      m_pLocUdpSocket;
    CCriticalSection    m_LocCritSect;

    CLocDataSet m_RxLocDataSet;
    CLocDataSet m_TxLocDataSet;
    CLocDataSet m_PreRxLocDataSet;
	// dq VISION
    CLocDataSet m_CamLocDataSet;
    std::deque<CLocDataSet>m_VecCamData;
    USHORT m_LocMode_Cam = 2;
private:
    std::string m_strVersion;
    unsigned int    m_uLocalPort;
    unsigned int    m_uRemotePort;
    STPoseData      m_PoseDataRecord;
    unsigned long long m_LastPoseTime;

public:
    // Default constructor
    CRoboLocClnt() {}
    CRoboLocClnt(CString strRemoteIp, UINT uRemotePort, BOOL bFramed, BOOL bAscii);

    ~CRoboLocClnt();

    //friend MagicSingleton3<CRoboLocClnt>;

public:
    bool Initialize();

    // Install the Mvc protocol
    BOOL Install();

    void Reset();

    BOOL Create();

    bool Stop();

    bool IsUsed() {return m_bUsed;}

    // Create the udp communicate channel.
    bool CreateUdpCom();

    // The support routine for RLP protocol
    void SupportRoutine();

    void ProcessReceiveData();

    void ProcessNetStatus();

    void ProcessDataSend();

    // Support auto-positioning agent
    void SupportAutoPosAgent();

    // 设置新的工作模式
    void SetNewWorkMode(unsigned char uMode) {m_aNewWorkMode = uMode;}
    unsigned char GetNewWorkMode() {return m_aNewWorkMode.load();}

    void SetCurWorkMode(unsigned char uMode) {m_aCurWorkMode = uMode;}
    unsigned char GetCurWorkMode() {return m_aCurWorkMode.load();}

    int  GetPlsLayer(string ip,unsigned int& key);
    void UpdatePlsState(string ip,int layer,int state,int workstate,unsigned int key);

private:
    // 上报定位系统信息
    bool ReportLocInfo();

    // 设置定位系统属性数据
    bool SetLocAttr();

    // 上报定位系统属性数据
    bool ReportLocAttr();

    // 切换定位系统工作模式
    bool ChangeMode();

    // 上报定位系统工作模式
    bool ReportLocMode();

    // 设置机器人当前位姿
    bool SetRoboPosition();

    // 上报机器人位姿
    bool ReportRoboPose();

    // 处理心跳数据帧
    bool HandleHeartBeatFrame();

    // 激光切区设置
    bool SetPlsLayer();

    //激光pls状态报告
    bool ReportPlsState();

    // 处理同步时间数据帧
    bool HandleSyncTimeFrame();

    // 空闲
    bool LocStandBy();

    // 执行建图过程
    bool DoMapping();

    // 执行定位过程
    bool DoLocalization();

    // 执行扫描过程
    bool DoScan();

    // 进入空闲模式
    bool EnterStandByMode();

    // 进入建图模式
    bool EnterMappingMode();

    // 进入定位模式
    bool EnterLocalizationMode();

    // 进入扫描模式
    bool EnterScanMode();

    // 接收到服务器端的应答
    bool OnServerEcho();

    bool SyncTimeBlocked();

public:
    // 网络通讯是否堵塞.
    bool IsBlocked();

    bool GetLocalVel(short& Vx, short& Vy, short& Vtheta);

    bool GetRoboInitPosition(unsigned char& chMapId, double& fX, double& fY, double& fTheta, unsigned long long& lRawTime, short& setpose_bypad);

    bool SetRoboPose(int nX, int nY, int nTheta, int lX, int lY, int lTheta, int ldX, int ldY, int ldTheta, short uG, unsigned short uN,
                     unsigned long long lPosTime, short nErrCode, short lErrCode, unsigned char chPosMode);

    bool SetRoboPose(int nX, int nY, int nTheta, short uG, unsigned short uN,
                     unsigned long long lPosTime, short nErrCode, unsigned char chPosMode);

    // dq VISION 发送重定位位姿到Cam
    bool SetInitPosition_Cam(STPositionData& positionData, int initmode);
    bool SetMappingMode_Cam();
    int  GetCamMode();
    bool SetLocMode_Cam();
    void SaveLocPosition_Cam();
    bool GetLocPosition_Cam(STPositionData& positionData);

    bool GetRxLocDataSet(CLocDataSet& data_set);

    int  GetReflectChargeFlag();

    bool GetSyncTimeData(STSyncTime& sync_time);

};

using RoboClntSingleton = MagicSingleton3<CRoboLocClnt>;

