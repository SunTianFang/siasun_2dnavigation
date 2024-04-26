//                                - CANMAN.H -
//
//   The interface of class "CCanManager".
//
//   Author: Zhanglei
//   Date:   2005. 5. 21
//

#pragma once

#include "Project.h"
#include "CanGyro.h"
#include "MagicSingleton.h"
#include "json/json.h"
#include <map>

#ifdef PLATFORM_WINDOWS_ARMV4I
#define CAN_PORT_COUNT	2
#else
#define CAN_PORT_COUNT	2
#endif

struct  GyroParam
{
    int nUnitID;
    int nCanID;
    bool bLostPowerInSleep;
};

//////////////////////////////////////////////////////////////////////////////
// The interface of class "CCanPort".
class DllExport CCanPort : public CCanChannel
{
public:

    CCanGyro    m_CanGyro;         // The CAN-Gyro protocol object

    friend class CCanManager;

    BOOL m_bEnableOutput;

public:
    CCanPort()
    {
        m_bEnableOutput = FALSE;
    }

    // Process all the received messages in a CAN port
    bool ProcessReceivedMsg();

    bool ProcessReceivedMsg(CCanMsg &Packet);

    // Refresh the inputs in a CAN port.
    void RefreshInputs();

    void SetSleepMode(BOOL bInSleep);

    bool InitCanGyro(int nCanId, int nUnitID, bool bLostPowerInSleep);

    //	friend DllExport CArchive& operator >> (CArchive& ar, CCanPort& Obj);
    //	friend DllExport CArchive& operator << (CArchive& ar, CCanPort& Obj);
};

//////////////////////////////////////////////////////////////////////////////
// The interface of class "CCanPort".
class DllExport CCanManager
{
public:
    CCanPort m_Port[3];         // Support the 2 CAN ports of PCM3680
    BOOL     m_bSimulate;
    BOOL     m_bInSleep;        // Flag: whether in sleep mode.
    BOOL     m_bWaked;          // Flag: whether truly waked.

    //test
    DWORD m_dwProtectCountOne;
    DWORD m_dwLastRecordCountOne;
    DWORD m_dwProtectCountTwo;
    DWORD m_dwProtectCountThird;
    DWORD m_dwLastRecordCountTwo;
    DWORD m_dwLastRecordCountThird;
    DWORD	m_dwLastRecordTimeOne;
    DWORD	m_dwLastRecordTimeTwo;
    DWORD	m_dwLastRecordTimeThird;
    BOOL m_bProtectOneStarted;
    BOOL m_bProtectTwoStarted;
    BOOL m_bProtectThirdStarted;
    int m_nProtectTime;
    CCriticalSection m_crit1;
    CCriticalSection m_crit2;
    CCriticalSection m_crit3;
    BOOL m_bReboot;

    BOOL m_bTimerTaskCrash;				//标示TimerTask 线程是否崩溃
    BOOL m_bTaskManagerProcCrash;		//标示TaskManagerProc线程是否崩溃
    BOOL m_bCanThreadProcCrash;			//标示CanThreadProc是否崩溃

    int m_nInitComStep;
    unsigned long long m_dwCompStart ;
    unsigned long long m_dwInitCom;
    HANDLE m_hGyroKillThread;       // Handle of "Kill thread" event
    HANDLE m_hGyroThreadDead;       // Handle of "Thread dead" event
    pthread_t m_GyroThread;



    HANDLE m_hKillThread;       // Handle of "Kill thread" event
    HANDLE m_hThreadDead;       // Handle of "Thread dead" event

#ifdef  LINUX_PLATFORM_USING
    pthread_t m_CanThread;
#endif

    bool m_bActivation;

    std::map<int, vector<GyroParam>>  GyroSettings;

private:
    CCanManager();

    friend MagicSingleton<CCanManager>;

public:
    ~CCanManager();

    bool Initialize();

    BOOL Install(BOOL bForAllDev = TRUE);
    void Start();
    void Stop();

    void Close();

    BOOL ReInit(int i);

    BOOL ReadVersion(WORD& uVersion);

    void SetSleepMode(BOOL bInSleep);


    BOOL LoadFile();
    void InitCanGyro();
    int InitGyroNaviOffset();//nStep = 11

    BOOL GetRelStatus(unsigned char& fRelStatus);

    void DriftCompensate(BOOL bStart);

    int InCompensate();

    BOOL GetRelAngle(CFloatDataInfo& RelAngle);

    bool GetGyroAngle(unsigned long long sync_time, CFloatDataInfo& gyro_angle);

    //	friend DllExport CArchive& operator >> (CArchive& ar, CCanManager& Obj);
    //	friend DllExport CArchive& operator << (CArchive& ar, CCanManager& Obj);
};

using CanManagerSingleton = MagicSingleton<CCanManager>;

