//                           - Wj719LaserScanner.H -
//
//   The interface of class "Wj719LaserScanner".
//   Author: liu wenzhi
//   Date:   2022. 07. 14
//

#ifndef CWJ_719_LASER_SCANNER_H
#define CWJ_719_LASER_SCANNER_H

#include "RangeScanner.h"
#include "Tools.h"
#include "async_client.h"
#include "wj_719_lidar_protocol.h"
using namespace std;
using namespace wj719_lidar;

class cWJ719LaserScanner : public CRangeScanner
{
  public:
        cWJ719LaserScanner(int fAngRes, float fStartAng, float fEndAng,CLaserScannerParam *cParam);

        ~cWJ719LaserScanner();

        virtual bool Stop();

        virtual BOOL Start(const char*device_name, \
                         const char*host_name = NULL, \
                         const int laserid = 0, \
                         const int iPort = 2110, \
                         const int iNetType = TCP_DATA, \
                         const int iScanFrequency = 25/*20*/, \
                         const int iSamplesPerScan = 1081);

  public:
        void SupportMeasure();

        BOOL ConnectToWJ719Laser(string host_name, int port);

        BOOL CloseWJ719LaserConnection();

        void CallBackRead(unsigned char* data, const int len);

  public:
        HANDLE                              m_hKillThread;       // Handle of "Kill thread" event
        HANDLE                              m_hThreadDead;       // Handle of "Thread dead" event

  private:
        /*wj719_lidar::*/wj_719_lidar_protocol  *m_pLidarProtocol;
        Async_Client              *m_pAsyncClient;
        pthread_t                           m_WJ719ScanThread;
        std::string                         m_LaserScannerIp;
        int                                 m_iLaserPort;
        int                                 m_iSamplesPerScan;
        int                                 m_iNetType; //1:Tcp 2:Udp

        vector<float>                       m_lastData;
        vector<float>                       m_lastIntensity;

};
#endif // CWJ_719_LASER_SCANNER_H
