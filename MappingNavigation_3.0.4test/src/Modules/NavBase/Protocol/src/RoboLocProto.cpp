//                                - CRoboLocProto.CPP -
//
//   Implementatin of class "CRoboLocProto".
//

#include "stdafx.h"
#include "RoboLocProto.h"
#include "UdpChannel.h"
#include "Tools.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


///////////////////////////////////////////////////////////////////////////////
//
//   Initialize the variables.
//
void CRoboLocProto::Initialize()
{
    nVersion = 1;
}

//
//   Clear the variables.
//
void CRoboLocProto::Clear()
{
}

// Deserialize:反序列化；Serialize：序列化
// Client interface
// 上报定位系统信息.
bool CRoboLocProto::ClntReportLocInfoSe(CUdpChannel* pChannel, STLocInfo& locInfo)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_LOCINFO
              << locInfo.ID
              << locInfo.nSwVersion
              << locInfo.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportLocInfoDe(CUdpChannel* pChannel, STLocInfo& locInfo)
{
    bool bResult = false;
    STLocInfo locInfo_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locInfo_.ID
            >> locInfo_.nSwVersion
            >> locInfo_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        locInfo_.lRecvTime = GetTickCount();
        locInfo_.bUpdate = true;
        locInfo = locInfo_;
        bResult = true;
    }
    return bResult;
}

// 应答定位系统属性命令.
bool CRoboLocProto::ClntReplyAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPLY_ATTR
              << locAttr.nRadiusFrom
              << locAttr.nRadiusTo
              << locAttr.NClosest
              << locAttr.uReserve1
              << locAttr.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReplyAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locAttr_.nRadiusFrom
            >> locAttr_.nRadiusTo
            >> locAttr_.NClosest
            >> locAttr_.uReserve1
            >> locAttr_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        locAttr_.lRecvTime = GetTickCount();
        locAttr_.bUpdate = true;
        locAttr = locAttr_;
        bResult = true;
    }
    return bResult;
}

// 上报定位系统属性.
bool CRoboLocProto::ClntReportAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_ATTR
              << locAttr.nRadiusFrom
              << locAttr.nRadiusTo
              << locAttr.NClosest
              << locAttr.uReserve1
              << locAttr.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locAttr_.nRadiusFrom
            >> locAttr_.nRadiusTo
            >> locAttr_.NClosest
            >> locAttr_.uReserve1
            >> locAttr_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        locAttr_.lRecvTime = GetTickCount();
        locAttr_.bUpdate = true;
        locAttr = locAttr_;
        bResult = true;
    }
    return bResult;
}

// 上报同步时间.
bool CRoboLocProto::ClntReportSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_SYNC_TIME
              << syncTime.lSrvTime
              << syncTime.lClntTime
              << syncTime.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> syncTime_.lSrvTime
            >> syncTime_.lClntTime
            >> syncTime_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        syncTime = syncTime_;
        bResult = true;
    }
    return bResult;
}

// 获取同步时间.
bool CRoboLocProto::ClntGetSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_GET_SYNC_TIME
              << syncTime.lClntTime
              << syncTime.lTimeOffset
              << syncTime.uFlag
              << syncTime.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntGetSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> syncTime_.lClntTime
            >> syncTime_.lTimeOffset
            >> syncTime_.uFlag
            >> syncTime_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        syncTime = syncTime_;
        bResult = true;
    }
    return bResult;
}

//应答工作模式命令.
bool CRoboLocProto::ClntReplyModeSe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPLY_MODE
              << operateMode.chMode
              << operateMode.chErrCode
              << operateMode.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReplyModeDe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> operateMode_.chMode
            >> operateMode_.chErrCode
            >> operateMode_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        operateMode_.bUpdate = true;
        operateMode = operateMode_;
        bResult = true;
    }
    return bResult;
}

// 上报工作模式.
bool CRoboLocProto::ClntReportModeSe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_MODE
              << operateMode.chMode
              << operateMode.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportModeDe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> operateMode_.chMode
            >> operateMode_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        operateMode_.bUpdate = true;
        operateMode = operateMode_;
        bResult = true;
    }
    return bResult;
}

// 应答设置位姿命令.
bool CRoboLocProto::ClntReplyPositionSe(CUdpChannel* pChannel, STPositionData& positionData)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPLY_POSITION
              << positionData.chMapId
              << positionData.nX
              << positionData.nY
              << positionData.nTheta
              << positionData.lRawTime
              << positionData.uTimeSpan
              << positionData.uErrCode
              << positionData.uReserve1
              << positionData.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReplyPositionDe(CUdpChannel* pChannel, STPositionData& positionData)
{
    bool bResult = false;
    STPositionData positionData_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> positionData_.chMapId
            >> positionData_.nX
            >> positionData_.nY
            >> positionData_.nTheta
            >> positionData_.lRawTime
            >> positionData_.uTimeSpan
            >> positionData_.uErrCode
            >> positionData_.uReserve1
            >> positionData_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        positionData_.bUpdate = true;
        positionData = positionData_;
        bResult = true;
    }
    return bResult;
}

// 上报定位信息.
bool CRoboLocProto::ClntReportPoseSe(CUdpChannel* pChannel, STPoseData& poseData)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();

#ifdef USE_LEG_METHOD
    *pChannel << RLP_CLNT_REPORT_POSE
              << poseData.uErrCode
              << poseData.chWait
              << poseData.chMask
              << poseData.nX
              << poseData.nY
              << poseData.nTheta
              << poseData.uG
              << poseData.uN
              << poseData.GyroAngle
              << poseData.lGyroTime
              << poseData.chOutputMode
              << poseData.lRawTime
              << poseData.uTimeSpan
              << poseData.chPosMode
              << poseData.nInfoState
              << poseData.uReserve1
              << poseData.uReserve2
              << poseData.nLegX
              << poseData.nLegY
              << poseData.nLegTheta
              << poseData.nDiffLegX
              << poseData.nDiffLegY
              << poseData.nDiffLegTheta
              << poseData.uLegErrCode;
#else
    *pChannel << RLP_CLNT_REPORT_POSE
              << poseData.uErrCode
              << poseData.chWait
              << poseData.chMask
              << poseData.nX
              << poseData.nY
              << poseData.nTheta
              << poseData.uG
              << poseData.uN
              << poseData.GyroAngle
              << poseData.lGyroTime
              << poseData.chOutputMode
              << poseData.lRawTime
              << poseData.uTimeSpan
              << poseData.chPosMode
              << poseData.nInfoState
              << poseData.uReserve1
              << poseData.uReserve2;
#endif

    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportPoseDe(CUdpChannel* pChannel, STPoseData& poseData)
{
    bool bResult = false;
    STPoseData poseData_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> poseData_.uErrCode
            >> poseData_.chWait
            >> poseData_.chMask
            >> poseData_.nX
            >> poseData_.nY
            >> poseData_.nTheta
            >> poseData_.uG
            >> poseData_.uN
            >> poseData_.GyroAngle
            >> poseData.lGyroTime
            >> poseData_.chOutputMode
            >> poseData_.lRawTime
            >> poseData_.uTimeSpan
            >> poseData_.chPosMode
            >> poseData_.nInfoState
            >> poseData_.uReserve1
            >> poseData_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        poseData_.lRecvTime = GetTickCount();
        poseData_.bUpdate = true;
        poseData = poseData_;
        bResult = true;
    }
    return bResult;
}

// dq  VISION 发送速度信息
bool CRoboLocProto::ClntReportVel_Cam(CUdpChannel* pChannel, STVelocityData& velocityData)
{
    //std::cout<<"\033[32;1m^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ClntReportVel_Cam: "<<velocityData.Vx<<", "<<velocityData.Vy<<", "<<velocityData.Vtheta<<", "<<velocityData.lRawTime<<"\033[0m"<<std::endl;
    bool bResult = false;
    //pChannel->SetProtocolFormat(false,false);
    if (pChannel == NULL || !pChannel->InUse())
        return false;

    union
    {
        UCHAR uch[4];
        int n;
    }nData[3];
    nData[0].n = (int)velocityData.Vx;
    nData[1].n = (int)velocityData.Vy;
    nData[2].n = (int)velocityData.Vtheta;
    union
    {
        UCHAR uch[8];
        int n;
    }nData1[1];
    nData1[0].n =(uint64_t)velocityData.lGyroTime;
    //UCHAR uCheck = 0x0E ^ RLP_CLNT_REPORT_VEL_CAM;
    UCHAR uCheck = 0x16 ^ RLP_CLNT_REPORT_VEL_CAM;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            uCheck = uCheck^nData[i].uch[j];
        }
    }
    for(int i = 0; i < 8; i++)
    {
        uCheck = uCheck^nData1[0].uch[i];
    }
    *pChannel->GetChannelObject() << static_cast<UCHAR>(0xAC)
              << static_cast<UCHAR>(0xED)
              << static_cast<UCHAR>(22)//14
              << RLP_CLNT_REPORT_VEL_CAM
              << (int)velocityData.Vx
              << (int)velocityData.Vy
              << (int)velocityData.Vtheta
              << (uint64_t)velocityData.lGyroTime
              << uCheck;                     //check
    bResult = pChannel->DoSend_Cam();
   // std::cout<<"(uint64_t)velocityData.lRawTime: "<<(uint64_t)velocityData.lRawTime<<std::endl;
    pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq  VISION 发送重定位位姿到Cam
bool CRoboLocProto::ClntReportRelocPos_Cam(CUdpChannel* pChannel, STPositionData& positionData, int initmode)
{
    //std::cout<<"\033[32;1m$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ ClntReportRelocPos_Cam: "<<positionData.nX<<", "<<positionData.nY<<", "<<positionData.nTheta<<", "<<positionData.lRawTime<<", "<<initmode<<"\033[0m"<<std::endl;
    bool bResult = false;
    //pChannel->SetProtocolFormat(false,false);
    if (pChannel == NULL || !pChannel->InUse())
        return false;

    union
    {
        UCHAR uch[4];
        int n;
    }nData[3];
    nData[0].n = (int)positionData.nX;
    nData[1].n = (int)positionData.nY;
    nData[2].n = (int)positionData.nTheta;

    union
    {
        UCHAR uch[8];
        int n;
    }nData_timestamp[1];
    nData_timestamp[0].n =(uint64_t)positionData.lRawTime;

    //UCHAR uCheck = 0x0E ^ RLP_CLNT_REPORT_RelocPos_CAM;
    UCHAR uCheck = 0x17 ^ RLP_CLNT_REPORT_RelocPos_CAM;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            uCheck = uCheck^nData[i].uch[j];
        }
    }
    for(int i = 0; i < 8; i++)
    {
        uCheck = uCheck^nData_timestamp[0].uch[i];
    }
    if(initmode == 0)
    {
        uCheck = uCheck ^ 0x00;
        *pChannel->GetChannelObject()<<static_cast<UCHAR>(0xAC)
        //*pChannel << static_cast<UCHAR>(0xAC)
                  << static_cast<UCHAR>(0xED)
                  << static_cast<UCHAR>(23)     //14
                  << RLP_CLNT_REPORT_RelocPos_CAM
                  << (int)positionData.nX
                  << (int)positionData.nY
                  << (int)positionData.nTheta
                  << (uint64_t)positionData.lRawTime
                  << static_cast<UCHAR>(0x00)
                  << uCheck;                     //check
    }
    else
    {
        uCheck = uCheck ^ 0x01;
        *pChannel->GetChannelObject()<<static_cast<UCHAR>(0xAC)
        //*pChannel << static_cast<UCHAR>(0xAC)
                  << static_cast<UCHAR>(0xED)
                  << static_cast<UCHAR>(23)     //14
                  << RLP_CLNT_REPORT_RelocPos_CAM
                  << (int)positionData.nX
                  << (int)positionData.nY
                  << (int)positionData.nTheta
                  << (uint64_t)positionData.lRawTime
                  << static_cast<UCHAR>(0x01)
                  << uCheck;                     //check
    }
    bResult = pChannel->DoSend_Cam();
   // std::cout<<"Reloc_Cam Msg has been sent!!! "<<std::endl;
    //pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq VISION 发送Cam暂停定位模式
bool CRoboLocProto::ClntSetMappingMode_Cam(CUdpChannel* pChannel)
{
    std::cout<<"\033[33;1m-----SetMappingMode to Cam----\033[0m"<<std::endl;
    bool bResult = false;
   // pChannel->SetProtocolFormat(false,false);
    if (pChannel == NULL || !pChannel->InUse())
        return false;

    UCHAR uCheck = 0x03 ^ RLP_CLNT_REPORT_SETMAPPING_CAM ^ 0x00;
    uCheck = uCheck ^ static_cast<UCHAR>(0);
    *pChannel->GetChannelObject()<<static_cast<UCHAR>(0xAC)
    //*pChannel << static_cast<UCHAR>(0xAC)
              << static_cast<UCHAR>(0xED)
              << static_cast<UCHAR>(3)
              << RLP_CLNT_REPORT_SETMAPPING_CAM //12
              << static_cast<UCHAR>(0)
              << uCheck;                     //check

    bResult = pChannel->DoSend_Cam();
    std::cout<<"\033[33;1mSetMappingMode Msg has been sent to cam!!! \033[0m"<<std::endl;
   // pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq VISION 发送Cam启动定位模式
bool CRoboLocProto::ClntSetLocMode_Cam(CUdpChannel* pChannel)
{
    std::cout<<"\033[33;1m-----SetLocMode to Cam----\033[0m"<<std::endl;
    bool bResult = false;
   // pChannel->SetProtocolFormat(false,false);
    if (pChannel == NULL || !pChannel->InUse())
        return false;

    UCHAR uCheck = 0x03 ^ RLP_CLNT_REPORT_SETMAPPING_CAM ^ 0x01;
    uCheck = uCheck ^ static_cast<UCHAR>(0);
    *pChannel->GetChannelObject()<<static_cast<UCHAR>(0xAC)
    //*pChannel << static_cast<UCHAR>(0xAC)
              << static_cast<UCHAR>(0xED)
              << static_cast<UCHAR>(3)
              << RLP_CLNT_REPORT_SETMAPPING_CAM //12
              << static_cast<UCHAR>(1)
              << uCheck;                     //check

    bResult = pChannel->DoSend_Cam();
    std::cout<<"\033[33;1mSetLocMode Msg has been sent to cam!!! \033[0m"<<std::endl;
    // pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq  VISION 接受Cam位姿
bool CRoboLocProto::ClntGetPos_Cam(CUdpChannel* pChannel, STPositionData& positionData)
{
    bool bResult = false;
    STPositionData positionData_;
    UCHAR uchFlag;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    positionData_.chMapId = 0;
    *pChannel->GetChannelObject() >> positionData_.nX
              >> positionData_.nY
              >> positionData_.nTheta
              >> uchFlag
              >> positionData_.lRawTime;
    // 计算check码
    union
    {
        UCHAR uch[4];
        int n;
    }nData[3];

    nData[0].n = positionData.nX;
    nData[1].n = positionData.nY;
    nData[2].n = positionData.nTheta;
    union
    {
        UCHAR uch[8];
        int n;
    }nData1[1];
    nData1[0].n =(uint64_t)positionData_.lRawTime;
    // pChannel->m_uchRxXor = 0x0F ^ RLP_CLNT_RECV_POS_CAM;
    pChannel->m_uchRxXor = 0x17 ^ RLP_CLNT_RECV_POS_CAM;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            pChannel->m_uchRxXor = pChannel->m_uchRxXor ^ nData[i].uch[j];
        }
    }
    pChannel->m_uchRxXor = pChannel->m_uchRxXor ^ uchFlag;
    for(int i = 0; i < 8; i++)
    {
        pChannel->m_uchRxXor = pChannel->m_uchRxXor^nData1[0].uch[i];
    }
    // positionData_.uErrCode = (uchFlag == 0) ? 0:1;  //(0:failed 1:success)
    positionData_.uErrCode = uchFlag;
 //   std::cout<<"\033[34;1mpositionData_.uErrCode: "<<positionData_.uErrCode<<"\033[0m"<<std::endl;
    // std::cout<<"positionData_.lRawTime : "<<positionData_.lRawTime<<std::endl;
    // positionData_.lRawTime = GetTickCount();
    // std::cout<<"positionData_.lRawTime111:  "<<positionData_.lRawTime<<std::endl;
    //std::cout<<"\033[34;1m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RecvPos_Cam:"<<positionData_.nX<<", "<<positionData_.nY<<", "<<positionData_.nTheta<<", == time: "<<positionData_.lRawTime<<"\033[0m"<<std::endl;
    if(pChannel->CheckRxEndLANXIN())
    {
        positionData_.bUpdate = true;
        positionData = positionData_;
        bResult = true;
    }
    //pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq VISION 接受Cam重定位消息
bool CRoboLocProto::ClntGetRelocPos_Cam(CUdpChannel* pChannel, STPositionData& positionData)
{
   // std::cout<<"CRoboLocProto::ClntGetRelocPos_Cam"<<std::endl;
    bool bResult = false;
    STPositionData positionData_;
    UCHAR uchFlag;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    positionData_.chMapId = 0;

    *pChannel->GetChannelObject() >> positionData_.nX
              >> positionData_.nY
              >> positionData_.nTheta
              >> uchFlag;

    // 计算check码
    union
    {
        UCHAR uch[4];
        int n;
    }nData[3];

    nData[0].n = positionData.nX;
    nData[1].n = positionData.nY;
    nData[2].n = positionData.nTheta;

    pChannel->m_uchRxXor = 0x0F ^ RLP_CLNT_RECV_RELOCPOS_CAM;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            pChannel->m_uchRxXor = pChannel->m_uchRxXor ^ nData[i].uch[j];
        }
    }
    pChannel->m_uchRxXor = pChannel->m_uchRxXor ^ uchFlag;

    //positionData_.uErrCode = (uchFlag == 0) ? 0:1;//(0:failed 1:success)
    positionData_.uErrCode = uchFlag;
    positionData_.lRawTime = GetTickCount();

    if(pChannel->CheckRxEndLANXIN())
    {
        positionData_.bUpdate = true;
        positionData = positionData_;
        bResult = true;
    }
    //pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// dq VISION 接受Cam定位模式
bool CRoboLocProto::ClntGetMode_Cam(CUdpChannel* pChannel , USHORT& LocMode_Cam)
{
    std::cout<<"<< ============ ClntGetMode_Cam ============ >>"<<std::endl;
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel->GetChannelObject() >> LocMode_Cam;
     std::cout<<"ClntGetMode_Cam LocMode_Cam: "<<LocMode_Cam<<std::endl;
    // 计算check码
    pChannel->m_uchRxXor = 0x03 ^ RLP_CLNT_RECV_LOCMODE_CAM ^ LocMode_Cam;

    if(pChannel->CheckRxEndLANXIN())
    {
        bResult = true;
    }
   // pChannel->SetProtocolFormat(false,true);
    return bResult;
}

// 发送心跳数据.
bool CRoboLocProto::ClntReportHeartBeatSe(CUdpChannel* pChannel, STHeartBeat& heartBeat)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_HEARTBEAT
              << heartBeat.lSrvTime
              << heartBeat.lClntTime
              << heartBeat.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::ClntReportHeartBeatDe(CUdpChannel* pChannel, STHeartBeat& heartBeat)
{
    bool bResult = false;
    STHeartBeat heartBeat_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> heartBeat_.lSrvTime
            >> heartBeat_.lClntTime
            >> heartBeat_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        heartBeat_.bUpdate = true;
        heartBeat = heartBeat_;
        bResult = true;
    }
    return bResult;
}

///////////////////////////////////////////////////////////////////////////////
// Deserialize:反序列化；Serialize：序列化.
//
// 读取定位系统信息.
//
bool CRoboLocProto::SrvGetLocInfoSe(CUdpChannel* pChannel, STLocInfo& locInfo)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_GET_LOCINFO
              << locInfo.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvGetLocInfoDe(CUdpChannel* pChannel, STLocInfo& locInfo)
{
    bool bResult = false;
    STLocInfo locInfo_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locInfo_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        locInfo = locInfo_;
        bResult = true;
    }
    return bResult;
}

//
// 设置定位系统属性.
//
bool CRoboLocProto::SrvSetAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_SET_ATTR
              << locAttr.nRadiusFrom
              << locAttr.nRadiusTo
              << locAttr.NClosest
              << locAttr.uReserve1
              << locAttr.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvSetAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locAttr_.nRadiusFrom
            >> locAttr_.nRadiusTo
            >> locAttr_.NClosest
            >> locAttr_.uReserve1
            >> locAttr_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        locAttr = locAttr_;
        bResult = true;
    }
    return bResult;
}

//
// 读取定位系统属性.
//
bool CRoboLocProto::SrvGetAttrSe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_GET_ATTR
              << locAttr.uReserve1
              << locAttr.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvGetAttrDe(CUdpChannel* pChannel, STLocAttr& locAttr)
{
    bool bResult = false;
    STLocAttr locAttr_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> locAttr_.uReserve1
            >> locAttr_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        locAttr = locAttr_;
        bResult = true;
    }
    return bResult;
}

//
// 设置时间同步.
//
bool CRoboLocProto::SrvGetSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_GET_SYNC_TIME
              << syncTime.lSrvTime
              << syncTime.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvGetSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> syncTime_.lSrvTime
            >> syncTime_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        syncTime = syncTime_;
        bResult = true;
    }
    return bResult;
}

//
// 上报同步时间.
//
bool CRoboLocProto::SrvReportSyncTimeSe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_REPORT_SYNC_TIME
              << syncTime.lClntTime
              << syncTime.lSrvTime
              << syncTime.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvReportSyncTimeDe(CUdpChannel* pChannel, STSyncTime& syncTime)
{
    bool bResult = false;
    STSyncTime syncTime_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> syncTime_.lClntTime
            >> syncTime_.lSrvTime
            >> syncTime_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        syncTime = syncTime_;
        bResult = true;
    }
    return bResult;
}

//
// 设置工作模式.
//
bool CRoboLocProto::SrvSetModeSe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_SET_MODE
              << operateMode.chMode
              << operateMode.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvSetModeDe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> operateMode_.chMode
            >> operateMode_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        operateMode = operateMode_;
        bResult = true;
    }
    return bResult;
}

//
// 读取工作模式.
//
bool CRoboLocProto::SrvGetModeSe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_GET_MODE
              << operateMode.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvGetModeDe(CUdpChannel* pChannel, STOperateMode& operateMode)
{
    bool bResult = false;
    STOperateMode operateMode_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> operateMode_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        operateMode = operateMode_;
        bResult = true;
    }
    return bResult;
}

//
// 设置当前位姿.
//
bool CRoboLocProto::SrvSetPositionSe(CUdpChannel* pChannel, STPositionData& positionData)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_SET_POSITION
              << positionData.chMapId
              << positionData.nX
              << positionData.nY
              << positionData.nTheta
              << positionData.lRawTime
              << positionData.uTimeSpan
              << positionData.uReserve1
              << positionData.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvSetPositionDe(CUdpChannel* pChannel, STPositionData& positionData)
{
    bool bResult = false;
    STPositionData positionData_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> positionData_.chMapId
            >> positionData_.nX
            >> positionData_.nY
            >> positionData_.nTheta
            >> positionData_.lRawTime
            >> positionData_.uTimeSpan
            >> positionData_.uReserve1
            >> positionData_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        positionData_.lRecvTime = GetTickCount();
        positionData = positionData_;
        bResult = true;
    }
    return bResult;
}

//
// 获取定位信息.
//
bool CRoboLocProto::SrvGetPoseSe(CUdpChannel* pChannel, STVelocityData& velocityData)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_GET_POSE
              << velocityData.Vx
              << velocityData.Vy
              << velocityData.Vtheta
              << velocityData.SteerAngle
              << velocityData.HeadingAngle
              << velocityData.GyroAngle
              << velocityData.lGyroTime
              << velocityData.lRawTime
              << velocityData.coordBase
              << velocityData.Wait
              << velocityData.Mask
              << velocityData.uReserve1
              << velocityData.uReserve2;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvGetPoseDe(CUdpChannel* pChannel, STVelocityData& velocityData)
{
    bool bResult = false;
    STVelocityData velocityData_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> velocityData_.Vx
            >> velocityData_.Vy
            >> velocityData_.Vtheta
            >> velocityData_.SteerAngle
            >> velocityData_.HeadingAngle
            >> velocityData_.GyroAngle
            >> velocityData_.lGyroTime
            >> velocityData_.lRawTime
            >> velocityData_.coordBase
            >> velocityData_.Wait
            >> velocityData_.Mask
            >> velocityData_.uReserve1
            >> velocityData_.uReserve2;

    if (pChannel->CheckRxEnd())
    {
        velocityData_.lRecvTime = GetTickCount();
        velocityData = velocityData_;
        bResult = true;
    }
    if((int)velocityData_.lRecvTime-(int)velocityData_.lGyroTime > 10000)
        bResult = false;
    return bResult;
}

bool CRoboLocProto::SrvSetPlsLayerDe(CUdpChannel* pChannel, STPlsLayer& plsLayer)
{
    bool bResult = false;
    STPlsLayer plsLayer_;
    if (pChannel == NULL || !pChannel->InUse())
        return false;
   *pChannel >> plsLayer_.iLaserIpLength;
    char ip[plsLayer_.iLaserIpLength + 1];
    for(int j = 0;j<plsLayer_.iLaserIpLength;j++)
    {
        *pChannel >> ip[j];
    }
    ip[plsLayer_.iLaserIpLength] = '\0';
    plsLayer_.laserIp = ip;
    *pChannel >> plsLayer_.chSetLayer;
    *pChannel >> plsLayer_.iKey;
  //  printf("SrvSetPlsLayerDe ip %s,chSetLayer plsLayer_.chSetLayer %d,iKey %d\n",ip,plsLayer_.chSetLayer,plsLayer_.iKey);
    if (pChannel->CheckRxEnd())
    {
        printf("pChannel->CheckRxEnd() !!!!!!!!!!!!\n");
        plsLayer_.lRecvTime = GetTickCount();
        plsLayer = plsLayer_;
        bResult = true;
    }
    return bResult;
}

bool CRoboLocProto::ClntReportPlsStateSe(CUdpChannel* pChannel, STPlsData& plsData)
{
    bool bResult = false;
    if (pChannel == NULL || !pChannel->InUse())
        return false;
    pChannel->InitTxPacket();
    *pChannel << RLP_CLNT_REPORT_PLS_STATE
              << plsData.iLaserNum;
    for(int i = 0;i < plsData.iLaserNum;i++)
    {
        *pChannel << plsData.vBaseData[i].iLaserIpLength;
        const char* ip = plsData.vBaseData[i].laserIp.c_str();
        for(int j = 0;j<plsData.vBaseData[i].iLaserIpLength;j++)
        {
            *pChannel << ip[j];
        }
        *pChannel << plsData.vBaseData[i].chLayer;
        *pChannel << plsData.vBaseData[i].chState;
        *pChannel << plsData.vBaseData[i].chWorkState;
        *pChannel << plsData.vBaseData[i].iKey;

    }
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

//
// 发送心跳数据.
//
bool CRoboLocProto::SrvSendHeartBeatSe(CUdpChannel* pChannel, STHeartBeat& heartBeat)
{
    bool bResult = false;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    pChannel->InitTxPacket();
    *pChannel << RLP_SRV_SEND_HEARTBEAT
              << heartBeat.lSrvTime
              << heartBeat.lClntTime
              << heartBeat.uReserve;
    pChannel->FinishTxPacket();

    bResult = pChannel->DoSend();
    return bResult;
}

bool CRoboLocProto::SrvSendHeartBeatDe(CUdpChannel* pChannel, STHeartBeat& heartBeat)
{
    bool bResult = false;
    STHeartBeat heartBeat_;

    if (pChannel == NULL || !pChannel->InUse())
        return false;

    *pChannel >> heartBeat_.lSrvTime
            >> heartBeat_.lClntTime
            >> heartBeat_.uReserve;

    if (pChannel->CheckRxEnd())
    {
        heartBeat = heartBeat_;
        bResult = true;
    }
    return bResult;
}

