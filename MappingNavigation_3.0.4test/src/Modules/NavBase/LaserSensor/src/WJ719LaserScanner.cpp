
#include"WJ719LaserScanner.h"
#include"LinuxSetting.h"
#include"Project.h"


void* WJ719LaserScanProc(LPVOID pParam)
{
     cWJ719LaserScanner* oLaserScanner = reinterpret_cast<cWJ719LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}


cWJ719LaserScanner::cWJ719LaserScanner(int fAngRes, float fStartAng, float fEndAng, CLaserScannerParam *cParam):
    CRangeScanner(fAngRes, fStartAng, fEndAng, WJ719)
{
    m_iNetType = -1;
    m_LaserScannerIp = "";
    m_iLaserPort = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
    m_pLidarProtocol =  new /*wj719_lidar::*/wj_719_lidar_protocol();
    auto func_read = boost::bind(&/*wj719_lidar::*/wj_719_lidar_protocol::dataProcess, m_pLidarProtocol, _1, _2);
    m_pAsyncClient = new Async_Client(func_read);
//    m_pAsyncClient = new Async_Client(m_pLidarProtocol, 2);
}


cWJ719LaserScanner::~cWJ719LaserScanner()
{
    Stop();
    if(m_pLidarProtocol != NULL)
        delete m_pLidarProtocol;
    m_pLidarProtocol = NULL;
    if(m_pAsyncClient != NULL)
        delete m_pAsyncClient;
    m_pAsyncClient = NULL;
}

bool cWJ719LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseWJ719LaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);

    PthreadJoin(m_WJ719ScanThread);

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

BOOL cWJ719LaserScanner::ConnectToWJ719Laser(string host_name, int port)
{
    std::cout << "&&&&&&&" << host_name << "  " << port << std::endl;
    return m_pAsyncClient->connect(host_name, port);
}


BOOL cWJ719LaserScanner::CloseWJ719LaserConnection()
{
    return m_pAsyncClient->disconnect();
}

// 启动激光扫描传感器
BOOL cWJ719LaserScanner::Start(const char*device_name,
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
    m_iLaserPort = 2110;
    m_iNetType = iNetType;
//    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = 20;
    m_nConnectTime = GetTickCount();
    //设置工作频率，此频率许和下位机设置相同
    m_pLidarProtocol->setConfig(m_nFrequency);
    if(!ConnectToWJ719Laser(m_LaserScannerIp, m_iLaserPort))
    {
        std::cerr <<"ConnectToWJ719Laser Failed: "<<device_name<< " m_iLaserPort: "<< m_iLaserPort << std::endl;
        return FALSE;
    }
    else
    {
       std::cout<<"ConnectToWJ719Laser OK"<<std::endl;
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
    if(pthread_create(&m_WJ719ScanThread, &attr, WJ719LaserScanProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cerr<<"Creat WJ719LaserScanSupport Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat WJ719LaserScanSupport Pthread OK"<<std::endl;
    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}
//
//	获取Scanner数据
//
void cWJ719LaserScanner::SupportMeasure()
{
//    std::cout << "******loop******" << std::endl;
    if(m_pAsyncClient->m_bConnected)
        m_pAsyncClient->recvData();
    else
        m_pAsyncClient->reconnect();
    if(m_pLidarProtocol->getFullState())
    {
        if(!m_pLidarProtocol->getSyncState())
        {
 //           std::cout << "*****" << GetTickCount() << std::endl;
//            Sleep(40);
//            return;
        }
        vector<float> data;
        vector<float> intensity;
        m_pLidarProtocol->getData(&data);
        m_pLidarProtocol->getIntensity(&intensity);
//        if(m_lastData == data)
//        {
//            std::cerr << __FUNCTION__ << " line:" << __LINE__ << " " << this << " ******ALL SAME******" << std::endl;
//        }
//        m_lastData = data;
//        m_lastIntensity = intensity;
//        std::cout << "m_lastData size: " << m_lastData.size() << std::endl;
//        std::cout << this << "   " <<data.at(500) << "  " << data.at(700) <<  "   " << this << std::endl;
//        std::cerr << "m_pAsyncClient: " << m_pAsyncClient << " " << data.size() << std::endl;
        unsigned long long sync_time = m_pLidarProtocol->getSyncTime();
        unsigned long long raw_time = m_pLidarProtocol->getRawTime();
        //std::cout<<"00000000000000000000000000----"<<data[500]<<std::endl;
        //719无同步时间戳
        AddRawPointCloud(data, intensity, raw_time, raw_time-40);
       // AddRawPointCloud(data, intensity, raw_time, sync_time); //wt_test 20230129
    }
//    else
//        std::cerr << "m_pLidarProtocol->getFullState(): " << m_pLidarProtocol->getFullState() << std::endl;
}

void cWJ719LaserScanner::CallBackRead(unsigned char* data, const int len)
{
    m_pLidarProtocol->dataProcess(data, len);
}

