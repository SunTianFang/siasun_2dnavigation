//	Robo localization Protocol
#pragma once

#include <string>
#include <vector>
#include "Tools.h"
#define RLP_SRV_GET_LOCINFO			static_cast<UCHAR>(101)
#define RLP_SRV_SET_ATTR			static_cast<UCHAR>(103)
#define RLP_SRV_GET_ATTR			static_cast<UCHAR>(105)
#define RLP_SRV_GET_SYNC_TIME			static_cast<UCHAR>(107)
#define RLP_SRV_REPORT_SYNC_TIME		static_cast<UCHAR>(109)
#define RLP_SRV_SET_MODE			static_cast<UCHAR>(111)
#define RLP_SRV_GET_MODE			static_cast<UCHAR>(113)
#define RLP_SRV_SEND_HEARTBEAT			static_cast<UCHAR>(115)
#define RLP_SRV_SET_POSITION			static_cast<UCHAR>(117)
#define RLP_SRV_GET_POSE			static_cast<UCHAR>(119)
#define RLP_SRV_SET_PLS_LAYER			static_cast<UCHAR>(121)

#define RLP_CLNT_REPORT_LOCINFO                 static_cast<UCHAR>(102)
#define RLP_CLNT_REPLY_ATTR                     static_cast<UCHAR>(104)
#define RLP_CLNT_REPORT_ATTR			static_cast<UCHAR>(106)
#define RLP_CLNT_REPORT_SYNC_TIME		static_cast<UCHAR>(108)
#define RLP_CLNT_GET_SYNC_TIME			static_cast<UCHAR>(110)
#define RLP_CLNT_REPLY_MODE			static_cast<UCHAR>(112)
#define RLP_CLNT_REPORT_MODE                    static_cast<UCHAR>(114)
#define RLP_CLNT_REPORT_HEARTBEAT		static_cast<UCHAR>(116)
#define RLP_CLNT_REPLY_POSITION                 static_cast<UCHAR>(118)
#define RLP_CLNT_REPORT_POSE			static_cast<UCHAR>(120)
#define RLP_CLNT_REPORT_PLS_STATE		static_cast<UCHAR>(122)
// dq VISION
#define RLP_CLNT_REPORT_VEL_CAM                 static_cast<UCHAR>(10)
#define RLP_CLNT_REPORT_RelocPos_CAM            static_cast<UCHAR>(11)
#define RLP_CLNT_REPORT_SETMAPPING_CAM          static_cast<UCHAR>(12)
#define RLP_CLNT_RECV_POS_CAM                   static_cast<UCHAR>(1)
#define RLP_CLNT_RECV_RELOCPOS_CAM              static_cast<UCHAR>(2)
#define RLP_CLNT_RECV_LOCMODE_CAM               static_cast<UCHAR>(4)

#define RLP_CLNT_COMMAND_ERROR			static_cast<UCHAR>(200)
#define RLS_CTRL_CYCLE                          static_cast<UINT>(50)   //50


#define RLP_SRV                      'R'
#define RLP_CLNT                     'N'


#define RLP_NET_COMM_TIMEOUT         600
#define RLP_NET_RECV_TIMEOUT         500
#define RLP_NET_SEND_CYCLE           300
#define RLP_KEY_DATA_SEND_CYCLE      50
#define RLP_SYNC_TIME_SEND_CYCLE     3000   //3s
#define RLP_SYNC_TIME_TOLERATE_SPAN  40   //40ms
#define RLP_SYNC_TIME_TIMEOUT        100000   //100s
#define RLP_GET_POSE_TIMEOUT         500

enum TRLPWorkMode
{
    RLP_MODE_POWERON = 0,
    RLP_MODE_STANDBY,
    RLP_MODE_MAPPING,
    RLP_MODE_LOCALIZATION,
    RLP_MODE_SCAN,
    RLP_MODE_AUTOMAPPING,
    RLP_MODE_STOPMAPPING,
    RLP_MODE_CALIBRATION
};

enum TRLPCoordBase
{
    RLP_LOCAL_COORDINATE = 0,
    RLP_GLOBAL_COORDINATE
};

enum TRLPPosWait
{
    RLP_INSTANTLY_LAST_POSE = 0,
    RLP_WAIT_NEXT_POSE,
    RLP_COMPLEX_POSE
};

enum TRLPPosMask
{
    RLP_ONLY_POSE = 0,
    RLP_POSE_REFLECTOR,
    RLP_POSE_SCAN,
    RLP_POSE_REFLECTOR_SCAN
};

enum TRLPPosMaskEx
{
    RLP_PATH_FOLLOW_OK = 0,
    RLP_PATH_FOLLOW_LOST_QUALITY,
    RLP_PATH_FOLLOW_LOST_ANGLE,
    RLP_PATH_FOLLOW_LOST_DIST
};

enum TRLPPosMode
{
    RLP_INITIAL_POSITIONING = 0,
    RLP_CONTINUOUS_POSITIONING,
    RLP_VIRTUAL_POSITIONING,
    RLP_POSITIONING_STOPPED,
    RLP_POSITION_INVALID
};

enum TRLPErrorCode
{
    RLP_NO_ERROR = 0,
    RLP_WRONG_OPERATE_MODE,
    RLP_NO_POSITION_AVAILABLE,
    RLP_ROBOT_TIMEOUT,
    RLP_LASER_TIMEOUT,
    RLP_GYRO_TIMEOUT,
    RLP_POSITIONING_FAILED,
    RLP_ROBOT_SYNCTIME_FAILED,
    RLP_LASER_SYNCTIME_FAILED,
    RLP_LOADING_MAP,
    RLP_LOADING_MAP_FAILED,
    RLP_GRIDSLAM_FAILED,
    RLP_GENERAL_ERROR
};

class STBaseData
{
public:
    bool bUpdate;
    unsigned long long lRecvTime;
    unsigned long long timeLastSend;

public:
    void Initialize()
    {
        bUpdate = false;
        lRecvTime = 0;
        timeLastSend = 0;
    }

    STBaseData()
    {
        Initialize();
    }

    ~STBaseData()
    {
        Clear();
    }

    void Clear()
    {
    }
};

class STLocInfo : public STBaseData
{
public:
    unsigned char ID;
    unsigned int nSwVersion;
    short uReserve;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        ID = 0;
        nSwVersion = 0;
        uReserve = 0;
    }

    STLocInfo()
    {
        Initialize();
    }
};

class STLocAttr : public STBaseData
{
public:
    unsigned int nRadiusFrom;
    unsigned int nRadiusTo;
    unsigned int NClosest;
    unsigned long long lRawTime;
    short uReserve1;
    short uReserve2;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        nRadiusFrom = 0;
        nRadiusTo = 0;
        NClosest = 0;
        lRawTime = 0;
        uReserve1 = 0;
        uReserve2 = 0;
    }

    STLocAttr()
    {
        Initialize();
    }
};

class STSyncTime : public STBaseData
{
public:
    unsigned long long lSrvTime;
    unsigned long long lClntTime;
    long long lTimeOffset;
    unsigned char uFlag;
    unsigned long long lCalibTime;
    short uReserve;
    bool  bEnable;
    bool  bRemote;
    unsigned int   nCycleTime;
    unsigned int   nTolerateSpan;
    unsigned int   nTimeOut;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        lSrvTime = 0;
        lClntTime = 0;
        lTimeOffset = 0;
        uFlag = 0;
        lCalibTime = 0;
        uReserve = 0;
        bEnable = false;
        bRemote = false;
        nCycleTime = 0;
        nTolerateSpan = 0;
        nTimeOut = 0;
    }

    STSyncTime()
    {
        Initialize();
    }

    STSyncTime(const STSyncTime& other)
    {
        this->bUpdate = other.bUpdate;
        this->lRecvTime = other.lRecvTime;
        this->timeLastSend = other.timeLastSend;

        this->lSrvTime = other.lSrvTime;
        this->lClntTime = other.lClntTime;
        this->lTimeOffset = other.lTimeOffset;
        this->uFlag = other.uFlag;
        this->lCalibTime = other.lCalibTime;
        this->uReserve = other.uReserve;
        this->bEnable = other.bEnable;
        this->bRemote = other.bRemote;
        this->nCycleTime = other.nCycleTime;
        this->nTolerateSpan = other.nTolerateSpan;
        this->nTimeOut = other.nTimeOut;
    }

    STSyncTime & operator=(const STSyncTime& other)
    {
        if (this != &other && &other != NULL)
        {
            this->bUpdate = other.bUpdate;
            this->lRecvTime = other.lRecvTime;
            this->timeLastSend = other.timeLastSend;

            this->lSrvTime = other.lSrvTime;
            this->lClntTime = other.lClntTime;
            this->lTimeOffset = other.lTimeOffset;
            this->uFlag = other.uFlag;
            this->lCalibTime = other.lCalibTime;
            this->uReserve = other.uReserve;
            this->bEnable = other.bEnable;
            this->bRemote = other.bRemote;
            this->nCycleTime = other.nCycleTime;
            this->nTolerateSpan = other.nTolerateSpan;
            this->nTimeOut = other.nTimeOut;
        }
        return (*this);
    }
};

class STOperateMode : public STBaseData
{
public:
    unsigned char chMode;
    unsigned char chErrCode;
    short uReserve;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        chMode = 0;
        chErrCode = 0;
        uReserve = 0;;
    }

    STOperateMode()
    {
        Initialize();
    }
};

class STPositionData : public STBaseData
{
public:
    unsigned char chMapId;
    int nX;
    int nY;
    int nTheta;
    unsigned long long lRawTime;
    short uTimeSpan;     // 时间差值,从设置初始位姿的时间到发送初始位姿数据的时间间隔,用于初始位姿数据补偿
    short uErrCode;
    short uReserve1;
    short uReserve2;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        chMapId = 0;
        nX = 0;
        nY = 0;
        nTheta = 0;
        lRawTime = 0;
        uTimeSpan = 0;
        uErrCode = 0;
        uReserve1 = 0;
        uReserve2 = 0;
    }

    STPositionData()
    {
        Initialize();
    }

    bool operator ==(const STPositionData& other) const
    {
        if(this->chMapId == other.chMapId && this->nX == other.nX && this->nY == other.nY
                && this->nTheta == other.nTheta && this->lRawTime == other.lRawTime) {
            return true;
        }
        else {
            return false;
        }
    }

    bool operator != (const STPositionData& other) const
    {
        return !(*this == other);
    }

};

class STVelocityData : public STBaseData
{
public:
    short Vx;
    short Vy;
    short Vtheta;
    short SteerAngle;
    short HeadingAngle;
    int GyroAngle;
    unsigned long long lGyroTime;
    unsigned long long lRawTime;
    unsigned char coordBase;
    unsigned char Wait;
    unsigned char Mask;
    short uReserve1;
    short uReserve2;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        Vx = 0;
        Vy = 0;
        Vtheta = 0;
        SteerAngle = 0;
        HeadingAngle = 0;
        GyroAngle = 0;
        lGyroTime = 0;
        lRawTime = 0;
        coordBase = 0;
        Wait = 0;
        Mask = 0;
        uReserve1 = 0;
        uReserve2 = 0;
    }

    STVelocityData()
    {
        Initialize();
    }
};

class STPoseData : public STBaseData
{
public:
    short uErrCode;
    unsigned char chWait;
    unsigned char chMask;
    int nX;
    int nY;
    int nTheta;
    short uG;
    unsigned short uN;
    int GyroAngle;
    unsigned long long lGyroTime;
    unsigned char chOutputMode;
    unsigned long long lRawTime;
    short uTimeSpan;     // 时间差值,从定位时获取的点云数据时间到发送定位数据的时间间隔,用于定位数据补偿
    unsigned char chPosMode;
    int nInfoState;
    short uReserve1;
    short uReserve2;

    // By Sam Add For LegMethod
    int nLegX;
    int nLegY;
    int nLegTheta;
    int nDiffLegX;
    int nDiffLegY;
    int nDiffLegTheta;
    short uLegErrCode;


public:
    void Initialize()
    {
        STBaseData::Initialize();
        uErrCode = 0;
        chWait = 0;
        chMask = 0;
        nX = 0;
        nY = 0;
        nTheta = 0;
        uG = 0;
        uN = 0;
        GyroAngle = 0;
        lGyroTime = 0;
        chOutputMode = 0;
        lRawTime = 0;
        uTimeSpan = 0;
        chPosMode = 0;
        nInfoState = 0;
        uReserve1 = 0;
        uReserve2 = 0;
    }

    STPoseData()
    {
        Initialize();
    }
};

class STHeartBeat : public STBaseData
{
public:
    unsigned long long lSrvTime;
    unsigned long long lClntTime;
    short uReserve;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        lSrvTime = 0;
        lClntTime = 0;
        uReserve = 0;
    }

    STHeartBeat()
    {
        Initialize();
    }
};

class STPlsBaseData
{
public:
    int iLaserIpLength;
    std::string laserIp;
    unsigned char chLayer;
    unsigned char chState;
    unsigned char chWorkState;
    unsigned int iKey;
public:
    void Initialize()
    {
        iLaserIpLength = 0;
        chLayer = 0;
        chState = 3;
        chWorkState = 1; //1 一切正常，2 激光断了， 3 切区错误
        laserIp = "";
        iKey = 0;
    }

    STPlsBaseData()
    {
        Initialize();
    }
    ~STPlsBaseData()
    {
    }
};

class STPlsData : public STBaseData
{
public:
    int iLaserNum;
    std::vector<STPlsBaseData> vBaseData;

public:
    void Initialize()
    {
        STBaseData::Initialize();
        iLaserNum = 0;
    }

    STPlsData()
    {
        Initialize();
    }
    ~STPlsData()
    {
        Clear();
    }
    void Clear()
    {
       vBaseData.clear();
    }
};

class STPlsLayer : public STBaseData
{
public:
    int iLaserIpLength;
    std::string laserIp;
    unsigned char chSetLayer;
    unsigned int iKey;
public:
    void Initialize()
    {
        STBaseData::Initialize();
        iLaserIpLength = 0;
        chSetLayer = 0;
        laserIp = "";
        iKey = 0;
    }

    STPlsLayer()
    {
        Initialize();
    }
    ~STPlsLayer()
    {
    }
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLocDataSet".
class CLocDataSet
{
public:
    bool m_bNetBlocked;
    unsigned long long m_tLastEcho;
    unsigned long long m_tLastSend;
    unsigned char m_uLocState;

public:
    STLocInfo m_LocInfo;
    STLocAttr m_LocAttr;
    STSyncTime m_SyncTime;
    STOperateMode m_OperateModeW;   // 设置操作模式
    STOperateMode m_OperateModeR;   // 读取操作模式
    STPositionData m_PositionData;
    STPoseData m_PoseData;
    STVelocityData m_VelocityData;
    STHeartBeat m_HeartBeat;
    STPlsData m_PlsData;
    STPlsLayer m_PlsLayer;

public:
    void Initialize()
    {
        m_bNetBlocked = false;
        m_tLastEcho = 0;
        m_tLastSend = 0;
        m_uLocState = RLP_MODE_STANDBY;

        m_LocInfo.Initialize();
        m_LocAttr.Initialize();
        m_SyncTime.Initialize();
        m_OperateModeW.Initialize();
        m_OperateModeR.Initialize();
        m_PositionData.Initialize();
        m_PoseData.Initialize();
        m_VelocityData.Initialize();
        m_HeartBeat.Initialize();
        m_PlsData.Initialize();
        m_PlsLayer.Initialize();
    }

    CLocDataSet()
    {
        Initialize();
    }

   virtual ~CLocDataSet()
    {
        Clear();
    }

    CLocDataSet(const CLocDataSet& other)
    {
        Initialize();
        this->m_bNetBlocked = other.m_bNetBlocked;
        this->m_tLastEcho = other.m_tLastEcho;
        this->m_tLastSend = other.m_tLastSend;
        this->m_uLocState = other.m_uLocState;

        this->m_LocInfo = other.m_LocInfo;
        this->m_LocAttr = other.m_LocAttr;
        this->m_SyncTime = other.m_SyncTime;
        this->m_OperateModeW = other.m_OperateModeW;
        this->m_OperateModeR = other.m_OperateModeR;
        this->m_PositionData = other.m_PositionData;
        this->m_PoseData = other.m_PoseData;
        this->m_VelocityData = other.m_VelocityData;
        this->m_HeartBeat = other.m_HeartBeat;
        this->m_PlsData = other.m_PlsData;
        this->m_PlsLayer = other.m_PlsLayer;
    }

    CLocDataSet & operator=(const CLocDataSet& other)
    {
        if (this != &other && &other != NULL)
        {
            Initialize();
            this->m_bNetBlocked = other.m_bNetBlocked;
            this->m_tLastEcho = other.m_tLastEcho;
            this->m_tLastSend = other.m_tLastSend;
            this->m_uLocState = other.m_uLocState;

            this->m_LocInfo = other.m_LocInfo;
            this->m_LocAttr = other.m_LocAttr;
            this->m_SyncTime = other.m_SyncTime;
            this->m_OperateModeW = other.m_OperateModeW;
            this->m_OperateModeR = other.m_OperateModeR;
            this->m_PositionData = other.m_PositionData;
            this->m_PoseData = other.m_PoseData;
            this->m_VelocityData = other.m_VelocityData;
            this->m_HeartBeat = other.m_HeartBeat;
            this->m_PlsData = other.m_PlsData;
            this->m_PlsLayer = other.m_PlsLayer;
        }
        return (*this);
    }

    void Clear()
    {
    }
};


class CUdpChannel;

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CRoboLocProto".
class CRoboLocProto
{
public:
    unsigned int nVersion;

public:
    CRoboLocProto()
    {
        Initialize();
    }

    ~CRoboLocProto()
    {
        Clear();
    }

    CRoboLocProto(const CRoboLocProto& other)
    {
        Initialize();
        this->nVersion = other.nVersion;
    }

    CRoboLocProto & operator=(const CRoboLocProto& other)
    {
        if (this != &other && &other != NULL)
        {
            Initialize();
            this->nVersion = other.nVersion;
        }
        return (*this);
    }

    void Initialize();

    void Clear();

    // Deserialize:反序列化；Serialize：序列化.
    // Client interface
    // 上报定位系统信息.
    bool ClntReportLocInfoSe(CUdpChannel* pChannel, STLocInfo& locInfo);
    bool ClntReportLocInfoDe(CUdpChannel* pChannel, STLocInfo& locInfo);

    // 应答定位系统属性命令.
    bool ClntReplyAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr);
    bool ClntReplyAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr);

    // 上报定位系统属性.
    bool ClntReportAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr);
    bool ClntReportAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr);

    // 上报同步时间
    bool ClntReportSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime);
    bool ClntReportSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime);

    // 获取同步时间.
    bool ClntGetSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime);
    bool ClntGetSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime);

    //应答工作模式命令.
    bool ClntReplyModeSe(CUdpChannel* pChannel, STOperateMode& operateMode);
    bool ClntReplyModeDe(CUdpChannel* pChannel, STOperateMode& operateMode);

    // 上报工作模式.
    bool ClntReportModeSe(CUdpChannel* pChannel, STOperateMode& operateMode);
    bool ClntReportModeDe(CUdpChannel* pChannel, STOperateMode& operateMode);

    // 应答设置位姿命令.
    bool ClntReplyPositionSe(CUdpChannel* pChannel, STPositionData& positionData);
    bool ClntReplyPositionDe(CUdpChannel* pChannel, STPositionData& positionData);

    // 上报定位信息.
    bool ClntReportPoseSe(CUdpChannel* pChannel, STPoseData& poseData);
    bool ClntReportPoseDe(CUdpChannel* pChannel, STPoseData& poseData);

    // dq VISION 发送速度信息
    bool ClntReportVel_Cam(CUdpChannel* pChannel, STVelocityData& velocityData);
    bool ClntReportRelocPos_Cam(CUdpChannel* pChannel, STPositionData& positionData, int initmode);
    bool ClntGetPos_Cam(CUdpChannel* pChannel, STPositionData& positionData);
    bool ClntGetRelocPos_Cam(CUdpChannel* pChannel, STPositionData& positionData);
    bool ClntSetMappingMode_Cam(CUdpChannel* pChannel);
    bool ClntSetLocMode_Cam(CUdpChannel* pChannel);
    bool ClntGetMode_Cam(CUdpChannel* pChannel, USHORT& LocMode_Cam);
    // 发送心跳数据.
    bool ClntReportHeartBeatSe(CUdpChannel* pChannel, STHeartBeat& heartBeat);
    bool ClntReportHeartBeatDe(CUdpChannel* pChannel, STHeartBeat& heartBeat);

    // Deserialize:反序列化；Serialize：序列化.
    // Server interface
    // 读取定位系统信息.
    bool SrvGetLocInfoSe(CUdpChannel* pChannel, STLocInfo& locInfo);
    bool SrvGetLocInfoDe(CUdpChannel* pChannel, STLocInfo& locInfo);

    // 设置定位系统属性.
    bool SrvSetAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr);
    bool SrvSetAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr);

    // 读取定位系统属性.
    bool SrvGetAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr);
    bool SrvGetAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr);

    // 设置时间同步.
    bool SrvGetSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime);
    bool SrvGetSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime);

    // 上报同步时间.
    bool SrvReportSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime);
    bool SrvReportSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime);

    // 设置工作模式.
    bool SrvSetModeSe(CUdpChannel* pChannel, STOperateMode& operateMode);
    bool SrvSetModeDe(CUdpChannel* pChannel, STOperateMode& operateMode);

    // 读取工作模式.
    bool SrvGetModeSe(CUdpChannel* pChannel, STOperateMode& operateMode);
    bool SrvGetModeDe(CUdpChannel* pChannel, STOperateMode& operateMode);

    // 设置当前位姿.
    bool SrvSetPositionSe(CUdpChannel* pChannel, STPositionData& positionData);
    bool SrvSetPositionDe(CUdpChannel* pChannel, STPositionData& positionData);

    // 获取定位信息.
    bool SrvGetPoseSe(CUdpChannel* pChannel, STVelocityData& velocityData);
    bool SrvGetPoseDe(CUdpChannel* pChannel, STVelocityData& velocityData);

    // dq VISION 获取定位
    bool SrvGetPoseSe_Cam(CUdpChannel* pChannel, STPoseData& poseData);

    // 发送心跳数据.
    bool SrvSendHeartBeatSe(CUdpChannel* pChannel, STHeartBeat& heartBeat);
    bool SrvSendHeartBeatDe(CUdpChannel* pChannel, STHeartBeat& heartBeat);

    //激光切区设置
    bool SrvSetPlsLayerDe(CUdpChannel* pChannel, STPlsLayer& plsLayer);

    // 上报pls状态.
    bool ClntReportPlsStateSe(CUdpChannel* pChannel, STPlsData& plsData);
};
