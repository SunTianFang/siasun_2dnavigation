//                           - cLeimouF30LaserScanner.h -
//
//   The interface of class "cLeimouF30LaserScanner".
//   Author: liu wenzhi
//   Date:   2022. 12. 14
//
#ifndef CLWIMOU_F30_LASER_SCANNER_H
#define CLWIMOU_F30_LASER_SCANNER_H
#include "leimou_f30_driver.h"
#include "RangeScanner.h"

class cLeimouf30LaserScanner : public CRangeScanner
{
public:
    cLeimouf30LaserScanner(int fAngRes, float fStartAng, float fEndAng, CLaserScannerParam *cParam);

    ~cLeimouf30LaserScanner();

    virtual bool Stop();

    virtual BOOL Start(const char*device_name,
                     const char*host_name = NULL,
                     const int laserid = 0,
                     const int iPort = 2110,
                     const int iNetType = TCP_DATA,
                     const int iScanFrequency = 25,
                     const int iSamplesPerScan = 1081);

public:
    void SupportMeasure();

    BOOL ConnectToLMF30Laser(string host_name, int port);

    BOOL CloseLMF30LaserConnection();

public:
    HANDLE                              m_hKillThread;       // Handle of "Kill thread" event
    HANDLE                              m_hThreadDead;       // Handle of "Thread dead" event

private:
    Intelly*                            m_pIntellyClient;
    pthread_t                           m_LMF30ScanThread;
    std::string                         m_LaserScannerIp;
    int                                 m_iLaserPort;
    int                                 m_iSamplesPerScan;
    int                                 m_iNetType; //1:Tcp 2:Udp

};
#endif //CLWIMOU_F30_LASER_SCANNER_H
