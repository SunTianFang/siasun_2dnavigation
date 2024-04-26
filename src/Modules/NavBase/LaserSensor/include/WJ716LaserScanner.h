//                           - cWJ716LaserScanner.h -
//
//   The interface of class "cWJ716LaserScanner".
//   Author: liu wenzhi
//   Date:   2022. 07. 16
//


#ifndef CWJ_716_LASER_SCANNER_H
#define CWJ_716_LASER_SCANNER_H

//#include <deque>
//#include "ZTypes.h"
#include "RangeScanner.h"
#include"Tools.h"
#include "async_client.h"

using namespace std;


class cWJ716LaserScanner : public CRangeScanner
{
  public:
        cWJ716LaserScanner(int fAngRes, float fStartAng, float fEndAng,CLaserScannerParam *cParam);

        ~cWJ716LaserScanner();

        virtual bool Stop();

        virtual BOOL Start(const char*device_name, \
                         const char*host_name = NULL, \
                         const int laserid = 0, \
                         const int iPort = 2110, \
                         const int iNetType = TCP_DATA, \
                         const int iScanFrequency = 25/*20*/, \
                         const int iSamplesPerScan = 1081);

  public:
        void    SupportMeasure();

        BOOL    ConnectToWJ716Laser(string host_name, int port);

        BOOL    CloseWJ716LaserConnection();

  public:
        HANDLE                      m_hKillThread;       // Handle of "Kill thread" event
        HANDLE                      m_hThreadDead;       // Handle of "Thread dead" event

  private:
        wj_716N_lidar_protocol      *m_pLidarProtocol;
        Async_Client                *m_pAsyncClient;
        pthread_t                   m_WJ716ScanThread;
        std::string                 m_LaserScannerIp;
        int                         m_iLaserPort;
        int                         m_iSamplesPerScan;
        int                         m_iNetType; //1:Tcp 2:Udp

        vector<float>               m_lastData;
        vector<float>               m_lastIntensity;

};
#endif // CWJ_716_LASER_SCANNER_H
