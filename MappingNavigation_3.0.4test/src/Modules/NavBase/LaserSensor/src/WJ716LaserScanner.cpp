//                           - cWJ716LaserScanner.cpp -
//
//   The interface of class "cWJ716LaserScanner".
//   Author: liu wenzhi
//   Date:   2022. 07. 16
//

#include"WJ716LaserScanner.h"
#include"LinuxSetting.h"
#include"Project.h"

#include "blackboxhelper.hpp"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

void* WJ716LaserScanProc(LPVOID pParam)
{
     cWJ716LaserScanner* oLaserScanner = reinterpret_cast<cWJ716LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}


cWJ716LaserScanner::cWJ716LaserScanner(int fAngRes, float fStartAng, float fEndAng, CLaserScannerParam *cParam):
    CRangeScanner(fAngRes, fStartAng, fEndAng, WJ716)
{
    m_iNetType = -1;
    m_LaserScannerIp = "";
    m_iLaserPort = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
    m_pLidarProtocol =  new wj_716N_lidar_protocol();
    auto func = boost::bind(&wj_716N_lidar_protocol::dataProcess, m_pLidarProtocol, _1, _2);
    m_pAsyncClient = new Async_Client(func);
//    m_pAsyncClient = new Async_Client(m_pLidarProtocol);
}


cWJ716LaserScanner::~cWJ716LaserScanner()
{
    Stop();
    if(m_pLidarProtocol != NULL)
        delete m_pLidarProtocol;
    m_pLidarProtocol = NULL;
    if(m_pAsyncClient != NULL)
        delete m_pAsyncClient;
    m_pAsyncClient = NULL;
}

bool cWJ716LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseWJ716LaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);

    PthreadJoin(m_WJ716ScanThread);

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

    m_bStarted = false;
    return true;
}

BOOL cWJ716LaserScanner::ConnectToWJ716Laser(string host_name, int port)
{
    std::cout << "&&&&&&&" << host_name << "  " << port << std::endl;
    return m_pAsyncClient->connect(host_name, port);
}

BOOL cWJ716LaserScanner::CloseWJ716LaserConnection()
{
    return m_pAsyncClient->disconnect();
}

// 启动激光扫描传感器
BOOL cWJ716LaserScanner::Start(const char*device_name,
                               const char*host_name,
                               const int laserid,
                               const int iPort,
                               const int iNetType,
                               const int iScanFrequency,
                               const int iSamplesPerScan)
{
    if (m_bStarted){
        return TRUE;
    }

    m_LaserScannerIp = device_name;  //192.168.0.2
    m_iLaserPort = 2110;
    m_iNetType = iNetType;
//    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = /*15*/25;
    m_nConnectTime = GetTickCount();
    //设置工作频率，此频率许和下位机设置相同
    m_pLidarProtocol->setConfig(m_nFrequency);
    if(!ConnectToWJ716Laser(m_LaserScannerIp, m_iLaserPort))
    {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "return FALSE1");
#endif
        std::cerr <<"ConnectToWJ716Laser Failed: "<<device_name<< " m_iLaserPort: "<< m_iLaserPort << std::endl;
        return FALSE;
    }
    else
    {
       std::cout<<"ConnectToWJ716Laser OK"<<std::endl;
    }

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
    {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "return FALSE2");
#endif
            return FALSE;

    }

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
    {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "return FALSE3");
#endif
        return FALSE;
    }

    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_WJ716ScanThread, &attr, WJ716LaserScanProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "return FALSE4");
#endif
        std::cerr<<"Creat WJ716LaserScanSupport Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat WJ716LaserScanSupport Pthread OK"<<std::endl;
    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}



//
//	获取Scanner数据
//
void cWJ716LaserScanner::SupportMeasure()
{
//    std::cout << "******loop******" << std::endl;
    if(m_pAsyncClient->m_bConnected)
        m_pAsyncClient->recvData();
    else
        m_pAsyncClient->reconnect();

    if(m_pLidarProtocol->getFullState())
    {
//#ifdef USE_BLACK_BOX
//            FILE_BlackBox(LocBox, "WJ scanner m_bfullscan is true");
//#endif
        if(!m_pLidarProtocol->getSyncState())
        {
            //std::cout << "*****" << GetTickCount() << std::endl;
//#ifdef USE_BLACK_BOX
//            FILE_BlackBox(LocBox, "WJ scanner m_bsyncstate is false");
//#endif
//            Sleep(40);
//            return;
        }
        vector<float> data;
        vector<float> intensity;
        m_pLidarProtocol->getData(&data);
        m_pLidarProtocol->getIntensity(&intensity);
        if(m_lastData == data)
        {
            std::cerr << __FUNCTION__ << " line:" << __LINE__ << " " << this << " ******ALL SAME******" << std::endl;
        }
        m_lastData = data;
        m_lastIntensity = intensity;
        unsigned long long sync_time = m_pLidarProtocol->getSyncTime();
        unsigned long long raw_time = (uint64_t)(GetTickCount());
        //synctime 补偿40ms
        //AddRawPointCloud(data, intensity, raw_time, sync_time-40);
        AddRawPointCloud(data, intensity, raw_time, raw_time-40);
    }
//    else
//        std::cerr << "m_pLidarProtocol->getFullState(): " << m_pLidarProtocol->getFullState() << std::endl;
}

