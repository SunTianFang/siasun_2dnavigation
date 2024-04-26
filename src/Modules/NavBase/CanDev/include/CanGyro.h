//                                  - CANGYRO.H -
//
//   The interface of class "CCanGryo", which defines the CAN-GYRO communication 
//   protocol.
//
//   Author: Maxifeng
//   Date:   2011.12.12
//

#ifndef __CCanGyro
#define __CCanGyro

#include <Afxmt.h>
#include "CanChannel.h"
#include "CanNode.h"
#include "TimeStamp.h"
#include "Tools.h"
#include <deque>
#include <mutex>

#define MAX_ANGLE_DIFFERENCE   (3.0f / 180.0f * PI)

///////////////////////////////////////////////////////////////////////////////
//   The interface of class "CCCanGyroUnit".

class DllExport CCanGyroUnit
{
private:
    CCanChannel* m_pCan;
    CCriticalSection m_CritSection;

    std::deque<CFloatDataInfo> gyro_buff_data;
    std::mutex gyro_buff_mtx;
    unsigned int gyro_buff_count;

public:
    USHORT       m_uCanID;
    int          m_nUnitID;
    DWORD        m_dwLastRefresh;

    BOOL         m_bDataUpdated;

    CFloatDataInfo m_Angle;
    float  m_fLastSpinAngle;
    float  m_fSpinProgReal;
    float  m_fSpinVelReal;

    //	CFloatDataInfo m_LastAngle;
    CBoolDataInfo  m_InCompensate;
    CByteDataInfo  m_Status;

    BOOL         m_bLostPowerInSleep;
    BOOL         m_bInSleep;
    friend class CCanGyro;

    LONG         m_lJumpCount;

    BOOL     m_bSpinSuspend;

private:
    void ProcessReceivedMsg(CCanMsg& Packet);

public:
    CCanGyroUnit();

    // The constructor
    CCanGyroUnit(USHORT uCanID, int nUnitID, BOOL bLostPowerInSleep);

    // Create the object
    BOOL Create(CCanChannel* pCan, USHORT uCanID, int nUnitID);

    void SetCanChannel(CCanChannel* pCan) {m_pCan = pCan;}

    // Read the deviation from the guide path
    BOOL ReadAngle(CFloatDataInfo& fAngle);
    BOOL ReadSpinProgAngle(float& fAngle);
    BOOL ReadSpinVel(float& fVel);

    // Read the deviation from the guide path
    BOOL ReadStatus(unsigned char& bStatus);

    BOOL RefreshTimeOut(DWORD dwTimeOut);
    void SetSleepMode(BOOL bInSleep);

    void DriftCompensate(BOOL bStart);
    BOOL SetSpinInitAngle();
    int InCompensate();

    BOOL SuspendSpinAngle(BOOL bSuspend);

    bool UpdateGyroData(CFloatDataInfo& gyro_data);

    bool GetSyncData(unsigned long long sync_time, CFloatDataInfo& sync_data);
};

///////////////////////////////////////////////////////////////////////////////
//   The interface of class "CCanGyro".
class DllExport CCanGyro
{
public:
    CCanChannel*    m_pCan;          // Pointer to the CAN channel
    CCanGyroUnit*  m_pUnit[2];      // Support maximum of 4 CAN-GUIDE units
    int             m_nUnitCount;    // The count of CANOpen guide units

public:
    CCanGyro();
    ~CCanGyro();

    //
    BOOL Init(CCanChannel* pCan);

    // Start the device
    void StartDevice();

    // Stop the device
    void StopDevice();

    // Add a new CAN-GUIDE unit to the system
    BOOL AddUnit(BOOL bLostPowerInSleep, USHORT uCanID, int nUnitID);

    // Delete a unit from the system
    BOOL DelUnit(int nUnitID);

    void RefreshInputs();

    // Check if a packet is a CAN-GUIDE message
    BOOL CheckPacket(CCanMsg& Packet);

    // Process received CAN-GUIDE message
    void ProcessReceivedMsg(CCanMsg& Packet);

    // Read the deviation from the guide path
    BOOL ReadAngle(int nUnitID, CFloatDataInfo& fAngle);

    bool GetSyncAngle(int nUnitID, unsigned long long sync_time, CFloatDataInfo& sync_angle);

    BOOL ReadSpinProgAngle(int nUnitID, float& fAngle);
    BOOL ReadSpinVel(int nUnitID, float& fVel);

    // Read the deviation from the guide path
    BOOL ReadStatus(int nUnitID, unsigned char& bStatus);

    void DriftCompensate(int nUnitID, BOOL bStart);
    int InCompensate(int nUnitID);
    BOOL SetSpinInitAngle(int nUnitID);

    USHORT RefreshTimeOut(DWORD dwTimeOut);

    void SetSleepMode(BOOL bInSleep);

    BOOL SuspendSpinAngle(int nUnitID, BOOL bSuspend);

    BOOL InitCanGyro(int nCanId, int nUnitID, bool bLostPowerInSleep);

    //	friend DllExport CArchive& operator >> (CArchive& ar, CCanGyro& Obj);
    //	friend DllExport CArchive& operator << (CArchive& ar, CCanGyro& Obj);
};
#endif
