//
//   The interface of class "wj_719_lidar_protocol".
//   Author: liu wenzhi
//   Date:   2022. 07. 14
//
#ifndef WJ_719_LIDAR_PROTOCOL_H
#define WJ_719_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>

using namespace std ;
namespace wj719_lidar
{
  #define MAX_LENGTH_DATA_PROCESS 200000


  class wj_719_lidar_protocol
  {
  private:
      typedef struct TagDataCache
      {
        unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
        unsigned int m_u32in;
        unsigned int m_u32out;
      }DataCache;
  public:
    wj_719_lidar_protocol();

    bool setConfig(const int freq);

    bool dataProcess(unsigned char *data,const int reclen);

    bool protocl(unsigned char *data,const int len);

    bool OnRecvProcess(unsigned char *data, int len);

    bool checkXor(unsigned char *recvbuf, int recvlen);

    void send_scan(const char *data,const int len);

    bool getFullState() const { return m_bfullscan; }

    unsigned long long getSyncTime() const { return m_synctime; }

    unsigned long long getRawTime() const { return m_rawtime; }

    bool getSyncState() const { return m_bsyncstate; }

    void getData(vector<float> *distance);

    void getIntensity(vector<float> *intensity);

    bool heartstate;

  private:
    unsigned char        data_[MAX_LENGTH_DATA_PROCESS];
    DataCache   m_sdata;
    unsigned int g_u32PreFrameNo;
    float g_d719scandata[3600];
    int d_n719ScanInden[3600];

    int g_n32ScanDatRcvdFrames;
    int g_n32ARMSendNo; //Frame Sequence Number
    int g_n32MotorSpeed;  //Motor Speed

    unsigned char g_byteScanFrameBank1; //Bank Num of ScanData
    unsigned char g_byteScanFrameBank2;
    unsigned char g_byteScanFrameBank3;
    unsigned char g_byteScanFrameBank4;
    unsigned char g_byteScanFrameBank5; //Bank Num of ScanData
    unsigned char g_byteScanFrameBank6;
    unsigned char g_byteScanFrameBank7;
    unsigned char g_byteScanFrameBank8;
    unsigned char g_byteScanFrameBank9; //Bank Num of ScanData
    unsigned char g_byteScanFrameBankA;
    unsigned char g_byteScanFrameBankB;
    unsigned char g_byteScanFrameBankC;

    unsigned char fScandata719[14400];
    int                 total_point;
    int                 freq_scan;
    bool                m_bfullscan;
    unsigned long long  m_synctime;
    unsigned long long  m_rawtime;
    unsigned long long  m_diffvalue;
    bool                m_bsyncstate;
  };

}// namespace wj719_lidar
#endif // WJ_719_LIDAR_PROTOCOL_H
