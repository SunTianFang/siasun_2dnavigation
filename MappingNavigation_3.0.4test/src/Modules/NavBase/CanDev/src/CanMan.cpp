// - CANMAN.CPP -
//
//   Implementation of class "CCanManager".
//
//   Author: Zhanglei
//   Date:   2005. 6. 1
//

#include "CanMan.h"
#include "Tools.h"
#include <unistd.h>
#include "Project.h"
#include "Debug.h"
#include"Tools.h"
#include <fstream>
#include "json/json.h"

#define CURRENT_STEP_SUCCEED              1
#define CURRENT_STEP_FAILED               0
#define CURRENT_STEP_RUN                  2

#define  USING_CAN_PORT     0


//
//   Process all the received messages in a CAN port.
//
bool CCanPort::ProcessReceivedMsg()
{
    CCanMsg Packet;
    if (!GetCanChannel()->ReceiveMsg(&Packet))
    return false;

    if (m_CanGyro.CheckPacket(Packet))
        m_CanGyro.ProcessReceivedMsg(Packet);

    return true;
}

//
//   Process all the received messages in a CAN port.
//
bool CCanPort::ProcessReceivedMsg(CCanMsg &Packet)
{
    //CCanMsg Packet;
//    if (!GetCanChannel()->ReceiveMsg(&Packet))
//    return false;

    if (m_CanGyro.CheckPacket(Packet))
        m_CanGyro.ProcessReceivedMsg(Packet);

    return true;
}
//
//   Refresh the inputs in a CAN port.
//
void CCanPort::RefreshInputs()
{
    m_CanGyro.RefreshInputs();
}

//
//   Set sleep mode.
//
void CCanPort::SetSleepMode(BOOL bInSleep)
{
    m_CanGyro.SetSleepMode(bInSleep);
}

bool CCanPort::InitCanGyro(int nCanId, int nUnitID, bool bLostPowerInSleep)
{
    m_CanGyro.InitCanGyro(nCanId, nUnitID, bLostPowerInSleep);
}

//CArchive& operator >> (CArchive& ar, CCanPort& Obj)
//{
//    Obj.m_CanPos.m_iPortId =  Obj.m_iPortId;  // sfe1012 add for fpga can 0  1

//    ar >> Obj.m_CanPos >> Obj.m_CanOpen >> Obj.m_CanGuide >> Obj.m_CanEncoder >> Obj.m_CanBattMeter >> Obj.m_CanSES >> Obj.m_CanBMS >> Obj.m_CanRFID>>Obj.m_CanOpenEncoder >> Obj.m_CanGyro >> Obj.m_CanManubox >> Obj.m_CanIO;

//    return ar;
//}

//CArchive& operator << (CArchive& ar, CCanPort& Obj)
//{
//    ar << Obj.m_CanPos << Obj.m_CanOpen << Obj.m_CanGuide << Obj.m_CanEncoder << Obj.m_CanBattMeter << Obj.m_CanSES << Obj.m_CanBMS << Obj.m_CanRFID <<Obj.m_CanOpenEncoder << Obj.m_CanGyro << Obj.m_CanManubox << Obj.m_CanIO;
//    return ar;
//}

//////////////////////////////////////////////////////////////////////
//   The thread procedure for the CAN manager.

//UINT CanThreadProc(LPVOID pParam)
void *CanThreadProc(void *pParam)
{

    #ifdef _Borax_LINUX32
         int iCanPortCount = CAN_PORT_COUNT - 1;  // Can0 is the nomal can device . Can1 is used by fpga
    #else
         #ifdef AGV_DESKTOP_TEST_MRC_CAN
            int iCanPortCount = 1;
         #else
            int iCanPortCount = CAN_PORT_COUNT;
         #endif
    #endif

    CCanManager* pCanManager = (CCanManager*)pParam;
    while(sem_trywait(pCanManager->m_hKillThread) != WAIT_OBJECT_0 )
    {
        for (int i = 0; i < iCanPortCount; i++)
        {

                CCanPort& Port = pCanManager->m_Port[i];
//                if(Port.GetChannelOKValue())
                {
                    // Process received data here
                    #ifdef _X86_LINUX64
                        while(Port.ProcessReceivedMsg());  // must receive all can package
                    #elif _E3845_LINUX64
                    CCanMsg Packet;
                    TPCANMsg CanMsg;
                    while(Port.GetCanChannel()->ReceiveMsg(&Packet, &CanMsg))
                    {
                        Port.ProcessReceivedMsg(Packet);
                    }
                    #else
                        while (Port.GetCanChannel()->GetRxCount() > 0)  //must receive all can package
                        Port.ProcessReceivedMsg();
                    #endif

        //             Process inputs refresh here
                    Port.RefreshInputs();

        //             Process outputs refresho here
//                    if(Port.m_bEnableOutput)
//                    Port.ProcessOutput();

                    if (!pCanManager->m_bInSleep)
                    pCanManager->m_bWaked = TRUE;
                }
        }

        Sleep(10);
    }

    SetEvent(pCanManager->m_hThreadDead);

    return NULL;
}
void *CanGyroNaviOffsetThreadProc(void *pParam) //
{
    CCanManager* pCanManager = (CCanManager*)pParam;
    while(sem_trywait(pCanManager->m_hGyroKillThread) != WAIT_OBJECT_0 )
    {
        int iRet = pCanManager->InitGyroNaviOffset();
        if( iRet != CURRENT_STEP_RUN)
        {
            if(iRet = CURRENT_STEP_SUCCEED)
            {
                std::cout<<"GyroNaviOffset CURRENT_STEP_SUCCEED "<<std::endl;
            }
            if(iRet = CURRENT_STEP_FAILED)
            {
                std::cout<<"GyroNaviOffset CURRENT_STEP_FAILED "<<std::endl;
            }
            break;
        }
        else
        {
            std::cout<<"GyroNaviOffset CURRENT_STEP_RUN "<<std::endl;
        }
        Sleep(200);
    }
    SetEvent(pCanManager->m_hGyroThreadDead);
    return NULL;
}
//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CCanManager".
CCanManager::CCanManager()
{
    m_bSimulate = TRUE;
    m_dwProtectCountOne = 0;
    m_dwProtectCountTwo = 0;
    m_dwProtectCountThird = 0;
    m_dwLastRecordCountOne = 0;
    m_dwLastRecordCountTwo = 0;
    m_dwLastRecordCountThird = 0;
    m_dwLastRecordTimeOne = 0;
    m_dwLastRecordTimeTwo = 0;
    m_dwLastRecordTimeThird = 0;
    m_bProtectOneStarted = FALSE;
    m_bProtectTwoStarted = FALSE;
    m_bProtectThirdStarted = FALSE;
    m_bTimerTaskCrash = FALSE;
    m_bTaskManagerProcCrash = FALSE;
    m_bCanThreadProcCrash = FALSE;
    m_bInSleep = FALSE;
    m_bWaked = TRUE;
    m_bReboot = FALSE;

    //ADD
    m_CanThread = 0;
    m_GyroThread = 0;
    m_nProtectTime = 0;
    m_nInitComStep = 0;

    #ifdef  _X86_LINUX64
        CCanDeviceSingleton::Init();//sfe1012 add
    #endif

    m_bActivation = false;
}
CCanManager::~CCanManager()
{

    if (m_hKillThread != NULL)
         SetEvent(m_hKillThread);
    if (m_hKillThread != NULL)
    CloseHandle(m_hKillThread);
    delete m_hKillThread;

//    if (m_hGyroKillThread != NULL)
//         SetEvent(m_hGyroKillThread);
//    if (m_hGyroKillThread != NULL)
//    CloseHandle(m_hGyroKillThread);
//    delete m_hGyroKillThread;

    if (m_hThreadDead != NULL)
    CloseHandle(m_hThreadDead);
    delete m_hThreadDead;

//	while (WaitForSingleObject(m_hThreadDead, 0) != WAIT_OBJECT_0);
#ifdef _X86_LINUX64
    CCanDeviceSingleton::UnInit();//sfe1012 add
#endif

}

void CCanManager::Start()
{
    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);

    m_hGyroKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hGyroKillThread != NULL);

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);

    m_bInSleep = FALSE;
    m_bWaked = TRUE;
    m_bReboot = FALSE;

   // AfxBeginThread(CanThreadProc, (LPVOID)this, THREAD_PRIORITY_ABOVE_NORMAL);
    /***************************************************************/
     pthread_attr_t attr;
     SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_ABOVE_NORMAL, attr);
     if(pthread_create(&m_CanThread,&attr,CanThreadProc,(void *)this) != 0)
     {
         std::cout<<"Creat CanThreadProc Pthread Failed"<<std::endl;
         return ;
     }
     std::cout<<"Creat CanThreadProc Pthread OK"<<std::endl;
     pthread_attr_destroy(&attr);
   /****************************************************************/
     pthread_attr_t attrt1;
     SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attrt1);
     if(pthread_create(&m_GyroThread,&attrt1,CanGyroNaviOffsetThreadProc,(void *)this) != 0)
     {
         std::cout<<"Creat CanGyroNaviOffsetThreadProc Pthread Failed"<<std::endl;
         return ;
     }
     std::cout<<"Creat CanGyroNaviOffsetThreadProc Pthread OK"<<std::endl;
     pthread_attr_destroy(&attrt1);
}

void CCanManager::Stop()
{
    if(m_hKillThread)
    SetEvent(m_hKillThread);

//    if(m_hGyroKillThread)
//    SetEvent(m_hGyroKillThread);

    if(m_hThreadDead)
    WaitForSingleObject(m_hThreadDead, 5000);

    if(m_hKillThread)
    CloseHandle(m_hKillThread);

//    if(m_hGyroKillThread)
//    CloseHandle(m_hGyroKillThread);

    if(m_hThreadDead)
    CloseHandle(m_hThreadDead);
}

void CCanManager::Close()
{
    int i = 0;

    for (i = 0; i < CAN_PORT_COUNT; i++)
     m_Port[i].Close();

}

bool CCanManager::Initialize()
{
    bool bRet = false;
    m_bActivation = true;

    LoadFile();
    // 初始化陀螺仪
    InitCanGyro();

    //开启Can线程 和 惯导 漂移补偿
    bRet = Install();
    return bRet;
}

BOOL CCanManager::Install(BOOL bForAllDev)
{
    int i;

    m_bSimulate = TRUE;
    m_dwProtectCountOne = 0;
    m_dwProtectCountTwo = 0;
    m_dwProtectCountThird = 0;
    m_dwLastRecordCountOne = 0;
    m_dwLastRecordCountTwo = 0;
    m_dwLastRecordCountThird = 0;
    m_dwLastRecordTimeOne = 0;
    m_dwLastRecordTimeTwo = 0;
    m_dwLastRecordTimeThird = 0;
    m_bProtectOneStarted = FALSE;
    m_bProtectTwoStarted = FALSE;
    m_bProtectThirdStarted = FALSE;
    m_bTimerTaskCrash = FALSE;
    m_bTaskManagerProcCrash = FALSE;
    m_bCanThreadProcCrash = FALSE;

   #ifdef _Borax_LINUX32
        int iCanPortCount = CAN_PORT_COUNT - 1;  // Can0 is the nomal can device . Can1 is used by fpga
   #else
        int iCanPortCount = CAN_PORT_COUNT;
   #endif

    for (i = 0; i < iCanPortCount; i++)
    {
        CCanChannel* pCanChannel = m_Port[i].GetCanChannel();

        if (pCanChannel == NULL)
        {

            return FALSE;
        }
#ifdef _X86_LINUX64
        if (!pCanChannel->Init(i+1, EMUC_BAUDRATE_250K))
        {
            return FALSE;
        }
#elif _E3845_LINUX64
        if (!pCanChannel->Init(i+1, CAN_BR_250K /*CAN_BR_500K*/))
        {
            return FALSE;
        }
#else
        if (!pCanChannel->Init(i, CAN_BR_250K))
        {
            return FALSE;
        }
#endif

       // m_Port[i].m_CanPos.Init(pCanChannel);

        if (bForAllDev)
        {
            m_Port[i].m_CanGyro.Init(pCanChannel);
        }
    }

   Start();
   //InitGyroNaviOffset();

   Sleep(50);

    m_bSimulate = FALSE;

    return TRUE;
}



BOOL CCanManager::ReInit(int i)
{
    CCanChannel* pCanChannel = m_Port[i].GetCanChannel();
    pCanChannel->Close();
#ifdef _X86_LINUX64
    return pCanChannel->Init(i+1, EMUC_BAUDRATE_250K);
#else
    return pCanChannel->Init(i, CAN_BR_250K);
#endif
}

BOOL CCanManager::ReadVersion(WORD& uVersion)
{
    int nReturn;
    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();

    nReturn = m_Port[iter->first - 1].GetVersion();
    if (nReturn  == -1)
        return FALSE;
    else
    {
        uVersion = (WORD)nReturn;
        return TRUE;
    }
}

void CCanManager::SetSleepMode(BOOL bInSleep)
{
    m_bInSleep = bInSleep;
    if (bInSleep)
        m_bWaked = FALSE;

    for (int i = 0; i < CAN_PORT_COUNT; i++)
        m_Port[i].SetSleepMode(bInSleep);
}

BOOL CCanManager::LoadFile()
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"CanParm.json");
    Json::Reader Jreader;
    Json::Value CanParmRoot;

    int iCanPortCount;
    int iGyroUnitCount, nUnitID;
    USHORT uCanID;
    BOOL   bLostPowerInSleep;
    vector<GyroParam>  vGyro;

    if(!FileLaserParm)
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, CanParmRoot))
    {
        if (!CanParmRoot["Port"].isNull())
        {
            iCanPortCount = CanParmRoot["Port"].size();
        }
        for(int i = 0; i < iCanPortCount;i++)
        {
            iGyroUnitCount = 0;
            if (!CanParmRoot["Port"][i]["GYRO"].isNull()) {
                iGyroUnitCount = CanParmRoot["Port"][i]["GYRO"].size();
            }
            for(int j = 0; j < iGyroUnitCount; j++)
            {
                GyroParam Gyrotemp;
                if (!CanParmRoot["Port"][i]["GYRO"][j]["nUnitID"].isNull()) {
                    nUnitID = CanParmRoot["Port"][i]["GYRO"][j]["nUnitID"].asInt();
                }
                else {
                    nUnitID = 0;
                }
                Gyrotemp.nUnitID = nUnitID;
                if (!CanParmRoot["Port"][i]["GYRO"][j]["uCanID"].isNull()) {
                    uCanID = CanParmRoot["Port"][i]["GYRO"][j]["uCanID"].asInt();
                }
                else {
                    uCanID = 70;
                }
                Gyrotemp.nCanID = uCanID;
                if (!CanParmRoot["Port"][i]["GYRO"][j]["bLostPowerInSleep"].isNull()) {
                    bLostPowerInSleep = CanParmRoot["Port"][i]["GYRO"][j]["bLostPowerInSleep"].asInt();
                }
                else{
                    bLostPowerInSleep = 1;
                }
                Gyrotemp.bLostPowerInSleep = bLostPowerInSleep;

                vGyro.push_back(Gyrotemp);
            }
            GyroSettings.insert(map<int, vector<GyroParam>>::value_type(iGyroUnitCount, vGyro));
        }
    }
    FileLaserParm.close();

    return bRet;
}
void CCanManager::InitCanGyro()
{
    map<int, vector<GyroParam>>::iterator iter;
    for(iter = GyroSettings.begin(); iter != GyroSettings.end(); ++iter)
    {
        int size = iter->second.size();
        vector<GyroParam> vgyrotemp;
        vgyrotemp = iter->second;
        cout<<"init gyro, size: "<<size<<", usring port:"<<iter->first - 1<<endl;
        for(int i = 0; i < size; i++)
        {
            m_Port[iter->first - 1].InitCanGyro(vgyrotemp[i].nCanID, vgyrotemp[i].nUnitID, vgyrotemp[i].bLostPowerInSleep);//todo
        }
        m_nProtectTime = 1000;
    }
}


BOOL CCanManager::GetRelStatus(unsigned char& fRelStatus)
{
    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();
    if (m_Port[iter->first - 1].m_CanGyro.ReadStatus(0,fRelStatus))
        return TRUE;
    else
        return FALSE;
}

void CCanManager::DriftCompensate(BOOL bStart)
{
    float fInterval;

    if (bStart)
        m_dwCompStart = GetTickCount();
    else
        fInterval = (GetTickCount() - m_dwCompStart) / 1000.0f;

    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();

    m_Port[iter->first - 1].m_CanGyro.DriftCompensate(0, bStart);
}

int CCanManager::InCompensate()
{
    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();

    return m_Port[iter->first - 1].m_CanGyro.InCompensate(0);
}

BOOL CCanManager::GetRelAngle(CFloatDataInfo& RelAngle)
{
    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();

    if (m_Port[iter->first - 1].m_CanGyro.ReadAngle(0, RelAngle))
        return TRUE;
    else
        return FALSE;
}

// 获取陀螺仪数据
bool CCanManager::GetGyroAngle(unsigned long long sync_time, CFloatDataInfo& gyro_angle)
{
    map<int, vector<GyroParam>>::iterator iter;
    iter = GyroSettings.begin();
    if (m_Port[iter->first - 1].m_CanGyro.GetSyncAngle(0, sync_time, gyro_angle))
        return TRUE;
    else
        return FALSE;
}

int CCanManager::InitGyroNaviOffset()//nStep = 11
{
#if !defined AGV_DEBUG
        if (true)
        {
                unsigned char bStatus;

                GetRelStatus(bStatus);

                if(TestBit(bStatus, 7) || TestBit(bStatus, 6) || TestBit(bStatus, 5) || TestBit(bStatus, 4))
                {
                }
                map<int, vector<GyroParam>>::iterator iter;
                iter = GyroSettings.begin();
                switch(m_nInitComStep)
                {
                case 0:

                        if (m_Port[iter->first - 1].m_CanGyro.InCompensate(0) == 0)
                        {
                                DriftCompensate(TRUE);
                        }
                        else
                        {
                                m_nInitComStep = 1;
                                m_dwInitCom = GetTickCount();
                        }
                        break;

                case 1:
                        if (GetTickCount() - m_dwInitCom > 15000)
                                return CURRENT_STEP_FAILED;
                        else if (GetTickCount() - m_dwInitCom > 7000)
                        {
                                if (InCompensate() == 1)
                                        DriftCompensate(FALSE);
                                else
                                {
                                        CFloatDataInfo fAngle;
                                        if (GetRelAngle(fAngle))
                                        {
                                                return CURRENT_STEP_SUCCEED;
                                        }
                                }
                        }

                        break;

                case 2:
                        break;
                }

                return CURRENT_STEP_RUN;
        }
        else
                return CURRENT_STEP_SUCCEED;
#else
        return CURRENT_STEP_SUCCEED;
#endif
}


