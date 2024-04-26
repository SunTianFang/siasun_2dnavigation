//                             - cLeimouF30LaserScanner.cpp -
//
//   The interface of class "cLeimouF30LaserScanner".
//   Author: liu wenzhi
//   Date:   2022. 12. 14
//
#include "LeimouF30LaserScanner.h"

void* Leimouf30LaserScanProc(LPVOID pParam)
{
     cLeimouf30LaserScanner* oLaserScanner = reinterpret_cast<cLeimouf30LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
        Sleep(50);
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}


cLeimouf30LaserScanner::cLeimouf30LaserScanner(int fAngRes, float fStartAng, float fEndAng, CLaserScannerParam *cParam):
    CRangeScanner(fAngRes, fStartAng, fEndAng, LEIMOUF30)
{
    m_iNetType = -1;
    m_LaserScannerIp = "";
    m_iLaserPort = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
    m_pIntellyClient = new Intelly();

}

cLeimouf30LaserScanner::~cLeimouf30LaserScanner()
{
    Stop();
    if(m_pIntellyClient != NULL)
    {
        delete m_pIntellyClient;
        m_pIntellyClient = NULL;
    }
}

bool cLeimouf30LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseLMF30LaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);

    PthreadJoin(m_LMF30ScanThread);

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

BOOL cLeimouf30LaserScanner::ConnectToLMF30Laser(string host_name, int port)
{
    std::cerr << "&&&&&&&" << host_name << "  " << port << std::endl;
//    return m_pAsyncClient->Connect(host_name, port);
    bool bRet = m_pIntellyClient->Connect(host_name, port);
    std::cerr << __LINE__ <<"======bRet: " << bRet << std::endl;

    if(bRet)
    {
        Sleep(1000);
        bRet = m_pIntellyClient->StartScan();
        std::cerr << __LINE__  << "======bRet: " << bRet << std::endl;

    }
    return bRet;
}


BOOL cLeimouf30LaserScanner::CloseLMF30LaserConnection()
{
    m_pIntellyClient->StopScan();
    return m_pIntellyClient->DisConnect();
}

// 启动激光扫描传感器
BOOL cLeimouf30LaserScanner::Start(const char*device_name,
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

    m_LaserScannerIp = device_name;  //192.198.0.2
    m_iLaserPort = 4001;
    m_iNetType = iNetType;
//    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = 30;

    m_nConnectTime = GetTickCount();

    if(!ConnectToLMF30Laser(m_LaserScannerIp, m_iLaserPort))
    {
        std::cerr <<"ConnectToLeimouf30Laser Failed: "<<device_name<< " m_iLaserPort: "<< m_iLaserPort << std::endl;
        return FALSE;
    }
    else
    {
       std::cout<<"ConnectToLeimouf30Laser OK"<<std::endl;
    }

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return FALSE;

    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_LMF30ScanThread, &attr, Leimouf30LaserScanProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cerr<<"Creat Leimouf30LaserScanProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat Leimouf30LaserScanProc Pthread OK"<<std::endl;
    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}
//
//	获取Scanner数据
//
void cLeimouf30LaserScanner::SupportMeasure()
{
    ScanData scan_data;
    bool bRet = m_pIntellyClient->GrabScanData(scan_data);
//    std::cerr << this->m_LaserScannerIp << "GetTickCount: " << GetTickCount() << "bRet: " << bRet << std::endl;

    if(m_pIntellyClient->is_connected() && bRet/*m_pIntellyClient->GrabScanData(scan_data)*/)
    {
        vector<uint32_t> data;
        vector<uint32_t> intensity;
        data.clear();
        intensity.clear();
//        std::cerr << this->m_LaserScannerIp << " " << GetTickCount() << " scan_data.num_values: " << scan_data.num_values << std::endl;
        for(uint32_t i = 0; i < scan_data.num_values; i++)
        {
            data.emplace_back(static_cast<uint32_t>(scan_data.ranges[i]));
            intensity.emplace_back(static_cast<uint32_t>(scan_data.intensities[i]));
        }
        if(data.empty()) return ;
//        std::cerr  << "data[0]: " << data.at(0) << std::endl;
        AddRawPointCloud(data, intensity, GetTickCount(), GetTickCount());
    }
    else
    {
//        std::cerr << this->m_LaserScannerIp <<"  m_pIntellyClient->is_connected(): "
//                  << m_pIntellyClient->is_connected() << "bRet: " << bRet << std::endl;
        return ;
    }

//    else
//        std::cerr << "m_pLidarProtocol->getFullState(): " << m_pLidarProtocol->getFullState() << std::endl;

    return ;
}
