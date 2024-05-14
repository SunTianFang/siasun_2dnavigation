//                           - LDS_50LaserScanner.H -
//
//   The interface of class "cLDS_E320LaserScanner".
//
//   Author: yuminghao
//   Date:   2024. 01. 2418
//

#ifndef CBLUDESEA_LASER_SCANNER_FR_H
#define CBLUDESEA_LASER_SCANNER_FR_H

#include <deque>
#include "ZTypes.h"
#include "RangeScanner.h"
#include "standard_interface.h"


#define DEFAULT_SCANNER_RESO_FR   0.1/*0.1f*/                               // 缺省角分辨率(度)
#define SCANNER_RESO_RAD_FR       (DEFAULT_SCANNER_RESO_FR/180.f*PI)  // 缺省角分辨率(弧度)
#define SCAN_COUNT_FR             ((int)(360/DEFAULT_SCANNER_RESO_FR))
#define MAX_LINE_NUM_FR           2000                            // 最大直线段数量
#define MAX_CIRCLE_NUM_FR         1000                           // 最大圆数量
#define MAX_DATA_COUNT_QUEUE_FR	18							//队列中数据最大数量

//网络数据类型
#define TCP_DATA 1
#define UDP_DATA 2

class cLDS_E320LaserScanner : public CRangeScanner
{
public:
        cLDS_E320LaserScanner(int fAngRes, float fStartAng, float fEndAng,CLaserScannerParam *cParam);
        ~cLDS_E320LaserScanner();

       virtual bool Stop();

public:
      void    SupportMeasure();
      // 启动激光扫描传感器
      virtual BOOL Start(const char*device_name,
                         const char*host_name = NULL,
                         const int laserid = 0,
                         const int iPort = 6543,
                         const int iNetType = TCP_DATA,
                         const int iScanFrequency = 30,
                         const int iSamplesPerScan = 3600);
      BOOL ConnectToLDSLaser(const char*device_name,const int iPort);
      BOOL CloseLDSLaserConnection();
public:
      //deque<vector<unsigned int>>pf;  //distance
      //deque<vector<unsigned int>>pfAmplitude; //Amplitude
      HANDLE      m_hKillThread;       // Handle of "Kill thread" event
      HANDLE      m_hThreadDead;       // Handle of "Thread dead" event


private:
      pthread_t   m_LDSScanThread;
      std::string m_LaserScannerIp;
      int         m_iLaserPort;
      int         m_iScanFrequency;
      int         m_iSamplesPerScan;
      int         m_iNetType; //1:Tcp 2:Udp
      uint64_t    m_nDiffValue;



};
#endif // CMAPPING_H
