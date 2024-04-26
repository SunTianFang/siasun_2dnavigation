//                           - PfR2000LaserScanner.cpp -
//
//   The interface of class "cPfR2000LaserScanner".
//
//   Author: sfe1012
//   Date:   2018. 01. 18
//

#include"FRR2LaserScanner.h"
#include"Tools.h"
//#include"Project.h"

void* FRR2000ScanProc(LPVOID pParam)
{
     cFRR2000LaserScanner* oLaserScanner = reinterpret_cast<cFRR2000LaserScanner*>(pParam);
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportMeasure();
        Sleep(20);
     }
     SetEvent(oLaserScanner->m_hThreadDead);
     pthread_exit(NULL);

   return NULL;
}

cFRR2000LaserScanner::cFRR2000LaserScanner(int fAngRes, float fStartAng, float fEndAng,CLaserScannerParam *cParam):
    CRangeScanner(fAngRes, fStartAng, fEndAng, FRR2)
{
    m_iNetType = -1;
    //m_R2000ScanThread = 0;
    m_LaserScannerIp = "";
    m_iLaserPort = 0;
    m_iScanFrequency = 0;
    m_iSamplesPerScan = 0;
    m_nLaserId = 0;
    m_bStarted = false;
    m_nDiffValue = GetRealTimeTickCount() - GetTickCount();

}

cFRR2000LaserScanner::~cFRR2000LaserScanner()
{
    Stop();
}

bool cFRR2000LaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    CloseR2000LaserConnection();

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);
    PthreadJoin(m_R2000ScanThread);
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

void cFRR2000LaserScanner::ReturnDist(int *GetData, const int &Size)
{
//    pthread_mutex_lock(&testlock);

//    std::cout<<">>>>>>>>>>m_NDist<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
//    for(int i = 0; i< 3598; i++)
//    {
//        std::cout<<m_nDist[i]<<" ";
//    }
//     std::cout<<">>>>>>>>>>m_NDist<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;

    memcpy(GetData,m_nDist,Size*sizeof(int));

//    pthread_mutex_unlock(&testlock);
}



BOOL cFRR2000LaserScanner::ConnectToR2000Laser()
{

    std::string scanner_ip = m_LaserScannerIp;

//    std::cout << "Connecting to scanner at " <<scanner_ip << " ... ";

    if (m_oR2000Driver.connect(scanner_ip, m_iLaserPort))//80
    {
        std::cout << "m_oR2000Driver.connect OK" << std::endl;
    }
    else
    {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << scanner_ip << " failed!" << std::endl;
        return FALSE;
    }

    m_oR2000Driver.setScanFrequency(30);

    m_oR2000Driver.setSamplesPerScan(m_iSamplesPerScan);

//    auto params = m_oR2000Driver.getParameters();

//    std::cout << "Current scanner settings:" << std::endl;
//    std::cout << "============================================================" << std::endl;
//    for (const auto& p : params)
//        std::cout << p.first << " : " << p.second << std::endl;
//    std::cout << "============================================================" << std::endl;

//    // Start capturing scanner data

//    std::cout << "Starting capturing: ";

    if( m_iNetType == UDP_DATA )
    {
        if (m_oR2000Driver.startCapturingUDP())
        {
            std::cout << " Udp OK " << std::endl;
        }
        else
        {
            std::cout << "Sorry R2000Laser startCapturingUDP FAILED! Now Try startCapturingTCP" << std::endl;
            if (m_oR2000Driver.startCapturingTCP())
            {
                std::cout << "OK!! Try R2000Laser startCapturingTCP OK " << std::endl;
            }
            else
            {
                std::cout << "Sorry!! Now Try startCapturingTCP Failed" << std::endl;
                return FALSE;
            }
        }
    }
    else if(m_iNetType == TCP_DATA)
    {
        if (m_oR2000Driver.startCapturingTCP())
        {
            std::cout << "R2000Laser startCapturingTCP OK" << std::endl;
        }
        else
        {
            std::cout << "Sorry R2000Laser startCapturingTCP FAILED! Now Try startCapturingUDP" << std::endl;
            if (m_oR2000Driver.startCapturingUDP())
            {
                std::cout << "OK R2000Laser startCapturingUDP OK " << std::endl;
            }
            else
            {
                std::cout << "Sorry Now Try startCapturingUDP Failed" << std::endl;
                return FALSE;
            }
        }
    }
    else
    {
        std::cout<<"Laser Net Type Not Tcp Or Udp"<<std::endl;
        return FALSE;
    }
    Sleep(1000);
   return TRUE;
}

BOOL cFRR2000LaserScanner::CloseR2000LaserConnection()
{
    std::cout << "Trying to stop capture" << std::endl;
    bool bRet = m_oR2000Driver.stopCapturing();
    std::cout << "Stopping capture: " << bRet << std::endl;
    m_oR2000Driver.disconnect();

    return TRUE;
}

BOOL cFRR2000LaserScanner::Start(const char*device_name,
                                 const char*host_name,
                                 const int laserid,
                                 const int iPort,
                                 const int iNetType,
                                 const int iScanFrequency,
                                 const int iSamplesPerScan)
{
    if (m_bStarted){
        return true;
    }

    m_LaserScannerIp = device_name;
    m_iLaserPort = iPort;
    m_iNetType = iNetType;
    m_iScanFrequency = iScanFrequency;
    m_iSamplesPerScan = iSamplesPerScan;
    m_nLaserId = laserid;
    m_nFrequency = iScanFrequency;
    m_nConnectTime = GetTickCount();

    if(!ConnectToR2000Laser())
    {
        std::cout<<"ConnectToR2000Laser Failed: "<<device_name<<std::endl;
        return FALSE;
    }
    else
    {
      std::cout<<"ConnectToR2000Laser OK"<<std::endl;
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

    if(pthread_create(&m_R2000ScanThread,&attr,FRR2000ScanProc,reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat R2000ScanSupport Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
        std::cout<<"Creat R2000ScanSupport Pthread OK"<<std::endl;

    pthread_attr_destroy(&attr);
    m_bStarted = true;

    return TRUE;
}

//
//	获取Urg Scanner数据
//
void cFRR2000LaserScanner::SupportMeasure()
{
    int scans_available = m_oR2000Driver.getFullScansAvailable();
    for (int i = 0; i < scans_available; i++)
    {		
		auto scandata = m_oR2000Driver.getFullScan();
//        std::cout<<"By Yu Test : scan_number = "<<scandata.distance_data.size()<<std::endl;

        if( scandata.distance_data.size() == (size_t)m_iSamplesPerScan )
        {



//            for(int i = 0; i < scandata.distance_data.size(); i++)
//            {
//                    if(scandata.amplitude_data.at(i) > 4090) {
//                       scandata.amplitude_data.at(i) = 255;
//                    }
//                    else
//                       scandata.amplitude_data.at(i) = ((scandata.amplitude_data.at(i))*255/4090);

//            }

            unsigned long long raw_time = GetTickCount();
            ASSERT(scandata.headers.size() == 12);
            pepperl_fuchs_fr::PacketHeader head = scandata.headers.at(0);
            uint32_t n32TimeStampSec = (uint32_t)((head.timestamp_sync & 0xFFFFFFFF00000000) >> 32);
            uint32_t n32TimeStampSubSec = (uint32_t)(head.timestamp_sync & 0x00000000FFFFFFFF);
            unsigned long long sync_time = NTPToUTC(n32TimeStampSec, n32TimeStampSubSec) - m_nDiffValue;
//            std::cerr << "n32TimeStampSec: " << n32TimeStampSec << " n32TimeStampSubSec: " << n32TimeStampSubSec << std::endl;
//            std::cerr << "sync_time: " << sync_time << " raw_time: " << raw_time << std::endl;
            AddRawPointCloud(scandata.distance_data, scandata.amplitude_data, raw_time, sync_time-33);
        }
        else
        {
            //std::cout<<"scandata.distance_data.size():"<<scandata.distance_data.size()<<std::endl;
        }
   }
}
