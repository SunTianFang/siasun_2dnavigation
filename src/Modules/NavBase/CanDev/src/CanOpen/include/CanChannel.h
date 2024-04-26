//                                  - CANCHANNEL.H -
//
//   The interface of class "CCanChannel".
//
//   Author: sfe1012
//   Date:   2016. 9. 2
//

#ifndef CCANCHANNEL_H
#define CCANCHANNEL_H

#ifdef _X86_LINUX64

//#define DEBUG_OUTPUT
extern "C" {

   #include "lib_emuc.h"

}

#define CAN_BR_125K                4	//EMUC_BAUDRATE_125K
#define CAN_BR_250K                5	//EMUC_BAUDRATE_250K
#define CAN_BR_500K                6	//EMUC_BAUDRATE_500K
#define CAN_BR_1M                  7	//EMUC_BAUDRATE_1M

#include"ZTypes.h"
#include"Geometry/Archive.h"
//pthread
#include <pthread.h>

#define MODEMDEVICE "/dev/ttyACM0"

#define STANDARD_FRAME 0x00  //Can Id 11bit
#define EXTEND_FRAME 0x01   //Can ID 29bit

enum
{
   SEND_DATA   = 0,
   RECV_DATA,
};

// Define for can port struct
struct  CCanConfig
{
        int   mode;              			// 0 : 11-bit;  1 : 29-bit
        DWORD accCode;
        DWORD accMask;
        int   baudrate;          		// 0 : 125KBps, 1 : 250KBps, 2 : 500KBps, 3 : 1MBps,
                                       // 4 : Self-Defined
        BYTE brp, tseg1, tseg2;	 	  // Used only if baudrate = 4
        BYTE sjw, sam;           	 // Used only if baudrate = 4
};

// Define can packet struct
struct  CCanMsg
{
    USHORT id;           //  CAN id
    unsigned char  rtr;          //  RTR bit
    unsigned char  dlen;         //  Data length
    unsigned char  m_uchData[8];      //  Data
};


/*********************  CCanDevice Single Class *************************************/
class CCanDeviceSingleton
{
    public:

        ~CCanDeviceSingleton();

        static CCanDeviceSingleton * GetSingleton()
        {
            if( NULL == s_lpSingleton )
            {
               s_lpSingleton = new CCanDeviceSingleton();
            }
            return s_lpSingleton;
        }

        static int Init();

        static int UnInit();

    private:
        CCanDeviceSingleton();
        //single object
        static CCanDeviceSingleton * s_lpSingleton;
    public:
        //Set Can Channel BaudRate
        int  SetCanChannelBaudRate(const int iChannel, const int iBaudRate);
        //Open Can Device
        bool  OpenCanDevice(const char *CanDevice);
        //Close Can Device
        void  CloseCanDevice();
        //Send Can Message
        int   SendMsg(const int &iChannel, const CCanMsg *pCanMsgPacket , int iMod);
        //Receive Can Message
        int   ReceiveMsg(const int &iChannel, CCanMsg *pCanMsgPacket);
    private:
        //Build AAEON LIB Can Message
        bool  BuildEmucPacket(const int &iChannel,const CCanMsg *pCanMsgPacket,DATA_INFO &package, int iMod);

        unsigned char char_2_hex(unsigned char c);

        unsigned char str_2_byte(unsigned char *c);

        void  update_log(DATA_INFO *info, int status);

        char  IntToChar(unsigned char input);

        bool  HexTo2Char(const unsigned char &cHex, char* OutTowChar );

        bool  CanMsgDataToChar(const int iLen,const unsigned char *packet,char *OutPut);

    private:
        int   m_hCanDeviceHandle;
};

/***********************************The interface of class "CCanChannel"*************************************************/
class  CCanChannel
{
public:
    int     m_nChannel; //EMUC_CH_1:(1) or EMUC_CH_2:(2)

    //HANDLE  m_hFile;
    CCanDeviceSingleton *m_pCanDevicesSingle;

    CCriticalSection m_CritSection;
    CCanMsg m_Packet;
    CCriticalSection m_NVRAMCritSec;

    int m_iMod;//sfe1012 add
public:
    CCanChannel()
    {
        //m_hFile = -1;
        m_nChannel = -1;
        m_iMod = STANDARD_FRAME;
        m_pCanDevicesSingle = CCanDeviceSingleton::GetSingleton();
    }

    virtual ~CCanChannel()
    {
        Close();
    }

    // Initialize the CAN channel
    bool Init(int nChannel, int nBaudRateType = CAN_BR_1M);

    // Close the CAN channel
    void Close();

    // Re-config the port
    bool Config(int nBaudRateType);

    // Get the number of packets received
    int GetRxCount();

    // Get the number of packets received
    int GetTxCount();

    // Send a packet of data
    bool SendMsg(CCanMsg* pPacket);

    // Receive a packet of data
    bool ReceiveMsg(CCanMsg *pPacket);

    // Callback function for customization
    virtual void OnSendMsg(CCanMsg* pPacket, bool bSuccess) {}

    // Callback function for customization
    virtual void OnReceiveMsg(CCanMsg* pPacket) {}

    // Enable/disable receive
    void EnableReceive(bool bYes = TRUE);

    // Read SJA1000 internal register
    unsigned char ReadRegister(unsigned char uchOffset);

    void WriteRegister(unsigned char uchOffset, unsigned char uchData);

    DWORD ReadErrorCount(unsigned char uchType);

    // Get the version of can driver
    int GetVersion();

    CCanChannel* GetCanChannel() {return this;}

    // Read the NVRAM
    USHORT ReadNVRAM(DWORD dwAddr);

    // Write to the NVARAM
    void WriteNVRAM(DWORD dwAddr, USHORT val);
};
#elif _E3845_LINUX64

#define CAN_BR_125K                125 * 1000	//EMUC_BAUDRATE_125K
#define CAN_BR_250K                250 * 1000	//EMUC_BAUDRATE_250K
#define CAN_BR_500K                500 * 1000	//EMUC_BAUDRATE_500K
#define CAN_BR_1M                  1000 * 1000	//EMUC_BAUDRATE_1M

#include <string.h>
#include "rsp/rsp_modules/rsp_define.h"
#include "rsp/rsp_dev/rsp_canDev_if.h"
//By Yu Add.
#include "rsp/rsp_schd_if.h"
#include "rsp/rsp_device_if.h"
// Define can packet struct
typedef struct  _CCanMsg
{
    unsigned int   id;           //  CAN id
    unsigned char  rtr;          //  RTR bit
    unsigned char  dlen;         //  Data length
    unsigned char  m_uchData[8]; //  Data
    _CCanMsg()
    {
        id = 0;
        rtr = 0;
        dlen = 0;
        memset(m_uchData , 0 , 8);
    }
    _CCanMsg(unsigned int id , unsigned char rtr ,unsigned char dlen)
    {
        this->id = id;
        this->rtr = rtr;
        this->dlen = dlen;
        memset(m_uchData , 0 , 8);
    }

}CCanMsg;

class CCanChannel
{
    public:

        CCanChannel();
        ~CCanChannel();

    public:
        // Initialize the CAN channel
        bool Init(int nChannel, int nBaudRateType = CAN_BR_500K);

        // Close the CAN channel
        virtual  bool Close();

        //Get the Channel
        CCanChannel* GetCanChannel() {return this;}

        void printf_head();

        int set_nonblocking(int &fd);

        // Get the number of packets received
        virtual int GetRxCount();

        // Get the number of packets received
        virtual int GetTxCount();

        // Send a packet of data
        virtual bool SendMsg(const CCanMsg *pPacket);

        // Receive a packet of data
        virtual bool ReceiveMsg(CCanMsg *pPacket,TPCANMsg *pTpPacket = NULL, int nMicroSecondsTimeOut = 2);

        // Enable/disable receive
        void EnableReceive(bool bYes = true);

        // Read SJA1000 internal register
        unsigned char ReadRegister(unsigned char uchOffset);

        void WriteRegister(unsigned char uchOffset, unsigned char uchData);

        unsigned long ReadErrorCount(unsigned char uchType);

        // Get the version of can driver
        int GetVersion();

        // Read the NVRAM
        unsigned short ReadNVRAM(unsigned long dwAddr);

        // Write to the NVARAM
        void WriteNVRAM(unsigned long dwAddr, unsigned short val);

        void printf_frame(const unsigned int frame_id, const unsigned char *data,
                          const unsigned char len, const bool extended,
                          const bool ok_flag,const bool sendflag);

    public:

      int   m_iChannel;

      RSP_HANDLE m_Canhandle; //CAN发送句柄

};

#else  //Mrc  And  Gsrd

#define CAN_BR_125K                125 * 1000	//EMUC_BAUDRATE_125K
#define CAN_BR_250K                250 * 1000	//EMUC_BAUDRATE_250K
#define CAN_BR_500K                500 * 1000	//EMUC_BAUDRATE_500K
#define CAN_BR_1M                  1000 * 1000	//EMUC_BAUDRATE_1M

#define TX_QUEUE_LEN		20*4096 // 使用足够多的发送缓存
#include"ZTypes.h"
#include"Archive.h"
#include "SocketCanLib/libsocketcan.h"

#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <stdint.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/types.h>
#include <linux/socket.h>

// Define can packet struct
typedef struct  _CCanMsg
{
    unsigned int   id;           //  CAN id
    unsigned char  rtr;          //  RTR bit
    unsigned char  dlen;         //  Data length
    unsigned char  m_uchData[8]; //  Data

#ifdef _Borax_LINUX32
    _CCanMsg()
    {
        id = 0;
        rtr = 0;
        dlen = 0;
        memset(m_uchData , 0 , 8);
    }
    _CCanMsg(unsigned int id , unsigned char rtr ,unsigned char dlen)
    {
        this->id = id;
        this->rtr = rtr;
        this->dlen = dlen;
        memset(m_uchData , 0 , 8);
    }
#endif

}CCanMsg;


class CCanChannel
{
    public:

        CCanChannel();
        ~CCanChannel();

    public:
        // Initialize the CAN channel
        bool Init(int nChannel, int nBaudRateType = CAN_BR_250K);

        // Close the CAN channel
        virtual  bool Close();

        //Get the Channel
        CCanChannel* GetCanChannel() {return this;}

        void printf_head();

        int set_nonblocking(int &fd);

        // Get the number of packets received
        virtual int GetRxCount();

        // Get the number of packets received
        virtual int GetTxCount();

        // Send a packet of data
        virtual bool SendMsg(const CCanMsg *pPacket);

        // Receive a packet of data
        virtual bool ReceiveMsg(CCanMsg *pPacket);

        // Enable/disable receive
        void EnableReceive(bool bYes = TRUE);

        // Read SJA1000 internal register
        unsigned char ReadRegister(unsigned char uchOffset);

        void WriteRegister(unsigned char uchOffset, unsigned char uchData);

        DWORD ReadErrorCount(unsigned char uchType);

        // Get the version of can driver
        int GetVersion();

        // Read the NVRAM
        USHORT ReadNVRAM(DWORD dwAddr);

        // Write to the NVARAM
        void WriteNVRAM(DWORD dwAddr, USHORT val);

        void printf_frame(const unsigned int frame_id, const unsigned char *data,
                          const unsigned char len, const bool extended,
                          const bool ok_flag,const bool sendflag);

     private:

        // Send a packet of data
        bool SocketCanSendMsg(can_frame &pPacket);

        // Receive a packet of data
        bool SocketCanReceiveMsg(can_frame *pPacket);

        void SetCanConfig(const int port, const int bitrate,const int iSendRegisterSize = TX_QUEUE_LEN);

        int set_can_loopback(const int sockfd, const bool lp);

        int socket_connect(const int port);

        void disconnect(int *sockfd);

        void CloseCanDev();

        void OpenCanDev();

        int send_frame(const int sockfd, const unsigned char* data, const int count);

        int recv_frame(const int sockfd, unsigned char* buf, const int count, const int timeout_ms);

        void close_socket(const int sockfd);

        int set_can_filter(const int iSockFd);

        int SetSocketSndBufSize(int &iSockFd , int iSize = 1024*1024);

        // 绑定sock，然后监听端口
        // 返回监听 套接字 文件描述符
        int socket_listen(const int port);

        void panic(const char *msg);

        //AGV CanMsg To Socket Can
        void CCanMsgToSCanFrameFun(const CCanMsg &AgvCanMsg, can_frame &SocketCanFrame);

        //Socket Can To AGV CanMsg
        void SCanFrameToCCanMsgFun(const can_frame &SocketCanFrame,CCanMsg &AgvCanMsg);

        void handle_err_frame(const struct can_frame *fr);

    private:

      char  pCanName[8];

      int   m_iChannel;

      int   m_BitRate;

      int   m_iSocketCanSendFd ;

      int   m_iSocketCanRevFd ;

      CCriticalSection m_CritSection;

      CCanMsg m_Packet;

      CCriticalSection m_NVRAMCritSec;

};
#endif  //_X86_LINUX64



#endif // CCANCHANNEL_H
