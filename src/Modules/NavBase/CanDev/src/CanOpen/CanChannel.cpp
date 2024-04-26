/*
// * CanChannel.cpp
// *
// *  Created on: 2017-9-2
// *      Author: sfe1012
// *
*/

#include "CanChannel.h"



#ifdef _X86_LINUX64
#include "Debug.h"
#include"BlackBox.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <signal.h>
#include <string>
#include"Tools.h"
extern "C" {
   #include "lib_emuc.h"
}
////for debug
#include<iostream>
using namespace std;
//
#define MODE_0       "Mode[11 bit]"
#define MODE_1       "Mode[29 bit]"
#define RTR_ENABLE   "RTR[ENABLE]"
#define RTR_DISABLE  "RTR[DISABLE]"
//
///* RS232 lib */
extern char comports[38][16];  //this is value in static LIB
CCanDeviceSingleton* CCanDeviceSingleton::s_lpSingleton = NULL;

#include "Debug.h"

#include "LogSystem.h"


//#ifndef AGV_DEBUG
//#define AGV_DEBUG
//#endif
//
///*****************************CCanDeviceSingleton Implementation*********************************************/
int CCanDeviceSingleton::Init()
{
    if( NULL == s_lpSingleton )
    {
       s_lpSingleton = new CCanDeviceSingleton();
    }
    return 0;
}
int CCanDeviceSingleton::UnInit()
{

    if(s_lpSingleton != NULL)
        delete s_lpSingleton;
    s_lpSingleton = NULL;

    return 0;
}
CCanDeviceSingleton::CCanDeviceSingleton()
{
    m_hCanDeviceHandle = -1;
    OpenCanDevice(MODEMDEVICE);
}
void  CCanDeviceSingleton::CloseCanDevice()
{
//  if(m_hCanDeviceHandle == 24)
//	EMUCCloseDevice(m_hCanDeviceHandle);
//	m_hCanDeviceHandle = -1;
}
CCanDeviceSingleton::~CCanDeviceSingleton()
{
//    if(m_hCanDeviceHandle == 24)
//        EMUCCloseDevice(m_hCanDeviceHandle);
//    m_hCanDeviceHandle = -1;

//    if(s_lpSingleton != NULL)
//        delete s_lpSingleton;
//    s_lpSingleton = NULL;
}
bool  CCanDeviceSingleton::OpenCanDevice(const char *CanDevice)
{

        #ifdef AGV_DEBUG
            return TRUE;
        #endif
      for(int i = 0; i < 38; i++)
       {
          if(strcmp(CanDevice, comports[i]) == 0)
          {
              m_hCanDeviceHandle = i;
              break;
          }
       }

       if(m_hCanDeviceHandle == -1)
       {
          perror(" Unknow COM m_hFile !! \n");
          return false;
       }
       else
       {
         if(!EMUCOpenDevice(m_hCanDeviceHandle))
          {
            #ifdef DEBUG_OUTPUT
                   std::cout<<"Open Can Device Ok!"<<std::endl;
            #endif
            Sleep(500);
            EMUCReset(m_hCanDeviceHandle);
            Sleep(500);
          }
          else
          {
            #ifdef DEBUG_OUTPUT
                   std::cout<<"Open Can Device Error!"<<std::endl;
            #endif
          }
       }
   return true;
}
int  CCanDeviceSingleton::SetCanChannelBaudRate(const int iChannel, const int iBaudRate)
{
     std::cout<<m_hCanDeviceHandle<<std::endl;
           EMUCReset(m_hCanDeviceHandle);
           std::cout<<"qq"<<std::endl;
   return  EMUCSetCAN(m_hCanDeviceHandle, iChannel, iBaudRate);

}
int  CCanDeviceSingleton::SendMsg(const int &iChannel, const CCanMsg *pCanMsgPacket, int iMod)
{
    #ifdef AGV_DEBUG
        return TRUE;
    #endif
    DATA_INFO package;
    BuildEmucPacket(iChannel,pCanMsgPacket,package,iMod);
    return EMUCSend(&package);
}
int  CCanDeviceSingleton::ReceiveMsg(const int &iChannel, CCanMsg *pCanMsgPacket)
{

    //add
    int iRet = 0;

    DATA_INFO data_recv;
    data_recv.com_port = m_hCanDeviceHandle;
    data_recv.channel = iChannel;

    iRet = EMUCReceive(&data_recv);

    if(iRet > 0)
    {
        //RTR
        if(data_recv.rtr)
            pCanMsgPacket->rtr = 1;
        else
            pCanMsgPacket->rtr = 0;

        //ID
        unsigned char IdTemp[5];
        if(data_recv.mod == 0)
        {
            IdTemp[0]=data_recv.id[3];
            IdTemp[1]=data_recv.id[2];
            IdTemp[2] = '\0';
            memcpy(&pCanMsgPacket->id,IdTemp,2);
        }
        else
        {
            IdTemp[0]=data_recv.id[3];
            IdTemp[1]=data_recv.id[2];
            IdTemp[2]=data_recv.id[1];
            IdTemp[3]=data_recv.id[0];
            IdTemp[4] = '\0';
            memcpy(&pCanMsgPacket->id,IdTemp,4);
        }
        //Len
        pCanMsgPacket->dlen = data_recv.dlc;

        if(data_recv.dlc > 8 || pCanMsgPacket->id > 0x7FF )
        {
            return -1;
        }

        //Data
        memset(pCanMsgPacket->m_uchData,0,8);

        if(data_recv.dlc > -1 && data_recv.dlc < 9)
        memcpy(pCanMsgPacket->m_uchData,data_recv.data,data_recv.dlc);


        #ifdef DEBUG_OUTPUT
                std::cout<<"iChannel:"<<iChannel<<std::endl;
                update_log(&data_recv, RECV_DATA);
        #endif

    }

    return iRet;
}
char  CCanDeviceSingleton::IntToChar(unsigned char input)
{
   char Ret[1];
   sprintf(Ret,"%X",input);
   return Ret[0];
}
bool  CCanDeviceSingleton::HexTo2Char(const unsigned char &cHex, char* OutTowChar )
{
    unsigned char cHexLow = cHex & 0x0f;
    unsigned char cHexHight = (cHex & 0xf0) >> 4;
    OutTowChar[0] = IntToChar(cHexHight);
    OutTowChar[1] = IntToChar(cHexLow);
    return true;
}
bool CCanDeviceSingleton::CanMsgDataToChar(const int iLen,const unsigned char *packet,char *OutPut)
{
    char T[2];
    int i = 0, j = 0;
    for ( i = 0, j = 0; i < iLen; i++ ,j += 2)
    {
        HexTo2Char(packet[i],T);
        OutPut[j] = T[0];
        OutPut[j+1] = T[1];
    }
    OutPut[j]= '\0';
    return true;
}
bool  CCanDeviceSingleton::BuildEmucPacket(const int &iChannel,const CCanMsg *pCanMsgPacket,DATA_INFO &package, int iMod)
{
       char tmp[2], id_tmp[8], data_tmp[16],inData[17];
       unsigned char hex;
       int i, j, len;
       package.com_port = m_hCanDeviceHandle;
       package.channel  = iChannel;
       package.mod      = iMod;   //(0:11bit, 1:29bit)  CanID
       package.rtr      = pCanMsgPacket->rtr;
       sprintf(id_tmp, "%08X", pCanMsgPacket->id);
       CanMsgDataToChar(pCanMsgPacket->dlen,pCanMsgPacket->m_uchData,inData);
       sprintf(data_tmp, "%s",  inData);
       len = strlen((char*) data_tmp);
       if(len % 2)
       {
          data_tmp[len] = 0x30;
          len += 1;
       }
       package.dlc = (len >> 1);
       for(i = 0; i < ((int)sizeof(id_tmp)); i++)
       {
          if(id_tmp[i] == 0x20)
             id_tmp[i] = 0x30;
       }
       if(package.mod == 0)
       {
          hex = char_2_hex(id_tmp[5]);
          sprintf(tmp, "%02X", hex & 0x07);
          id_tmp[4] = 0x00;
          id_tmp[5] = tmp[1];
       }
       else
       {
          hex = char_2_hex(id_tmp[0]);
          sprintf(tmp, "%02X", hex & 0x01);
          id_tmp[0] = tmp[1];
       }
       for(i=0, j=0; i<ID_LEN; i++, j+=2)
          package.id[i] = str_2_byte((unsigned char*)id_tmp+j);
       for(i=0, j=0; i<DATA_LEN; i++, j+=2)
          package.data[i] = str_2_byte((unsigned char*)data_tmp+j);

    #ifdef DEBUG_OUTPUT
        //Send Log
        update_log(&package, SEND_DATA);
    #endif
    return true;
}
unsigned char CCanDeviceSingleton::char_2_hex(unsigned char c)
{
   if (c >= 'A')
      return c - 'A' + 10;
   else
      return c - '0';
}
/*------------------------------------------------------------------------------------------------------------*/
unsigned char CCanDeviceSingleton::str_2_byte(unsigned char *c)
{
  return char_2_hex(c[0]) * 16 + char_2_hex(c[1]);
}
void CCanDeviceSingleton::update_log(DATA_INFO *info, int status)
{
   struct tm    *timeinfo;
   struct timeb now;

   static unsigned long send_cnt = 0;
   static unsigned long recv_cnt = 0;

   char str[128];
   int i = 0, n = 0, len = 0;

   ftime(&now);
   timeinfo = localtime((time_t*) &now);
   memset(str, 0, sizeof(str));

   if(status == SEND_DATA)
   {
      len = sprintf(str, "Send[%ld] %02d:%02d:%02d:%03d ", ++send_cnt,
                                                           timeinfo->tm_hour,
                                                           timeinfo->tm_min,
                                                           timeinfo->tm_sec,
                                                           now.millitm);
   }
   else
   {
      len = sprintf(str, "Recv[%ld] %02d:%02d:%02d:%03d ", ++recv_cnt,
                                                           timeinfo->tm_hour,
                                                           timeinfo->tm_min,
                                                           timeinfo->tm_sec,
                                                           now.millitm);
   }
   /* mode */
   if(info->mod)
      n = sprintf(str+len, "%s ", MODE_1);//extend frame
   else
      n = sprintf(str+len, "%s ", MODE_0);//normal frame
   len += n;
   /* rtr */
   if(info->rtr)
      n = sprintf(str+len, "%s ", RTR_ENABLE);
   else
      n = sprintf(str+len, "%s ", RTR_DISABLE);

   len += n;

   /* ID */
   n = sprintf(str+len, "ID[");
   len += n;

   for(i=0; i<ID_LEN; i++)
   {
      if(info->mod == 0 && i < 2)
         continue;

      n = sprintf(str+len, "%02X ", info->id[i]);
      len+=n;
   }

   len--;
   n = sprintf(str+len, "] ");
   len += n;

   /* DATA */
   if(info->rtr == 0)
   {
      n = sprintf(str+len, "DATA[");
      len += n;
      if(info->dlc > 8)
      {
          return ;
      }
      for(i=0; i<info->dlc; i++, len+=n)
         n = sprintf(str+len, "%02X ", info->data[i]);
      len--;
      n = sprintf(str+len, "]\n");
      len += n;
   }
   else
   {
      len--;
      n = sprintf(str+len, "\n");
      len += n;
   }
   printf("%s\n", str);
   //INFOLOG(str);

}

//#include "debug.h"
//#include "project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define IOCTL_CAN_GET_RX_PACKETS       1
#define IOCTL_CAN_SET_BAUDRATE         2
#define IOCTL_CAN_ENA_INT              3
#define IOCTL_CAN_READ_REG             4
#define IOCTL_CAN_WRITE_REG            5
#define IOCTL_CAN_ERR_STATUS           6
#define IOCTL_CAN_GET_TX_PACKETS       7

#define IOCTL_CAN_RX_HEAD              8
#define IOCTL_CAN_RX_TAIL              9
#define IOCTL_CAN_TX_HEAD              10
#define IOCTL_CAN_TX_TAIL              11

#define IOCTL_CAN_DRV_VERSION          12

#define IOCTL_NVRAM_READ             13
#define IOCTL_NVRAM_WRITE            14

USHORT  uCanTotalTxCount = 0;
USHORT  uCanTotalRxCount = 0;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CCanChannel".

//
//   Initialize the specified CAN channel.
//
BOOL CCanChannel::Init(int nChannel, int nBaudRateType)
{
    #ifdef AGV_DEBUG
        return FALSE;
    #else
        m_nChannel = nChannel;
        if (!Config(nBaudRateType))
            return FALSE;
        return TRUE;
   #endif
}

//
//   Close the CAN channel.
//
void CCanChannel::Close()
{
    if(m_pCanDevicesSingle != NULL)
    {
        m_pCanDevicesSingle->CloseCanDevice();
        m_pCanDevicesSingle = NULL;
    }
}

BOOL CCanChannel::Config(int nBaudRateType)
{
     std::cout<<"10 "<<std::endl;
    BOOL bResult;
    int iRet = m_pCanDevicesSingle->SetCanChannelBaudRate(m_nChannel, nBaudRateType);
//	if(iRet != 0 )
//	{
//		bResult = FALSE;
//	}
//	else
//		bResult = TRUE;
     std::cout<<"11 "<<std::endl;
    Sleep(100);

//	return bResult;
    return TRUE;
}

//
//   Send a packet of data through the CAN channel.
//
BOOL CCanChannel::SendMsg(CCanMsg *pPacket)
{
    #ifdef AGV_DEBUG
        return TRUE;
    #else

        BOOL   bResult;
        m_Packet = *pPacket;

        m_CritSection.Lock();
        int iRet = m_pCanDevicesSingle->SendMsg(m_nChannel , pPacket , m_iMod);
        m_CritSection.Unlock();

        if(iRet != 0 )
        {
            bResult = FALSE;
        }
        else
            bResult = TRUE;

        // Callback function for customization
        OnSendMsg(pPacket, bResult);



        return bResult;

    #endif
}

//
//   Receive a packet of data from the CAN channel.
//
BOOL CCanChannel::ReceiveMsg(CCanMsg* pPacket)
{
    #ifdef AGV_DEBUG
        return TRUE;
    #else
        m_CritSection.Lock();
        int iRet = m_pCanDevicesSingle->ReceiveMsg(m_nChannel , pPacket );
        m_CritSection.Unlock();

        BOOL bResult;
        if( iRet > 0)
        {
            bResult = TRUE;
        }
        else
            bResult = FALSE;

        // Callback function for customization
        if (bResult)
            OnReceiveMsg(pPacket);

        return bResult;
    #endif
}



//
//   Enable/disable receive
//
void CCanChannel::EnableReceive(BOOL bYes)
{

}


UCHAR CCanChannel::ReadRegister(UCHAR uchOffset)
{

        return -1;
}

//
//   Write to SJA1000 internal register.
//
void CCanChannel::WriteRegister(UCHAR uchOffset, UCHAR uchData)
{

}

//
//   Read SJA1000 internal error count.
//
DWORD CCanChannel::ReadErrorCount(UCHAR uchType)
{
    return -1;
}

//
//   Get the version of can driver
//
int CCanChannel::GetVersion()
{
        return -1;
}

//
//   Read the NVRAM.
//
USHORT CCanChannel::ReadNVRAM(DWORD dwAddr)
{
    USHORT uResult = -1;
    return uResult;
}

//
//   Write to the NVRAM.
//
void CCanChannel::WriteNVRAM(DWORD dwAddr, USHORT val)
{
}
//
//   Get the number of packets received
//
int CCanChannel::GetRxCount()
{
//	DWORD dwResult;
//
//	if (DeviceIoControl(m_hFile, IOCTL_CAN_GET_RX_PACKETS, NULL, 0, &dwResult, 4, NULL, NULL))
//		return (int)dwResult;
//	else
//        return -1;
}

//
//   Get the number of packets received
//
int CCanChannel::GetTxCount()
{
//	DWORD dwResult;
//
//	if (DeviceIoControl(m_hFile, IOCTL_CAN_GET_TX_PACKETS, NULL, 0, &dwResult, 4, NULL, NULL))
//		return (int)dwResult;
//	else
        return -1;
}
#elif _E3845_LINUX64

#include "rsp/rsp_schd_if.h"
#include "rsp/rsp_device_if.h"
#include <sys/time.h>
#include <time.h>
#include"linux/can.h"
#include <stdio.h>


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CCanChannel".

CCanChannel::CCanChannel()
{

}
CCanChannel::~CCanChannel()
{

}
//
//   Initialize the specified CAN channel.
//
bool CCanChannel::Init(int nChannel, int nBaudRateType)
{
    m_iChannel = nChannel;
    int nRtValue = RSP_CAN_Open(nChannel, &m_Canhandle);
    if (nRtValue != RSP_RT_SUCCESS)
    {
        printf("Can't open CAN %d\n", nChannel);
        return false;
    }
    else
    {
        printf("CAN %d OPEN SUCCEED\n", nChannel);
    }
    nRtValue = RSP_CAN_Config( m_Canhandle, (nBaudRateType/1000), CAN_INIT_TYPE_ST );
    if (nRtValue != RSP_RT_SUCCESS)
    {
        printf("Can't Configure CAN %d\n", nChannel);
        return false;
    }
    else
    {
        printf("CAN %d CONFIG SUCCEED\n", nChannel);
    }
    sleep(1);

    return true;
}

//
//   Close the CAN channel.
//
bool CCanChannel::Close()
{
     if(RSP_CAN_Close(m_Canhandle)!=RSP_RT_SUCCESS)
     {
         printf("Can't close CAN %d\n", m_iChannel);
        return false;
     }
     return true;
}

//
//   Send a packet of data through the CAN channel.
//
bool CCanChannel::SendMsg(const CCanMsg *pPacket)
{
    //return true;

    #ifdef AGV_DEBUG
        return TRUE;
    #else
    TPCANMsg oTPCANMsgTemp;
    oTPCANMsgTemp.ID = pPacket->id;
    oTPCANMsgTemp.MSGTYPE = pPacket->rtr;
    oTPCANMsgTemp.LEN = pPacket->dlen;
    memcpy(oTPCANMsgTemp.DATA,pPacket->m_uchData,8);
    int result = RSP_CAN_SendBlock(m_Canhandle, (TPCANMsg *)&oTPCANMsgTemp);

    usleep(300);

    if (result != RSP_RT_SUCCESS)
    {
        printf("CAN %d Send Failed\n", m_iChannel);
        #ifndef DEBUG_OUTPUT
            printf_frame(pPacket->id & CAN_EFF_MASK, pPacket->m_uchData, pPacket->dlen,
                        ((pPacket->id & CAN_EFF_FLAG) ? true : false),
                        false,
                        true);
        #endif
        return false;
    }
    else
    {
        #ifndef DEBUG_OUTPUT
           printf_frame(pPacket->id & CAN_EFF_MASK, pPacket->m_uchData, pPacket->dlen,
                       ((pPacket->id & CAN_EFF_FLAG) ? true : false),
                       true,
                       true);
        #endif
        return true;
    }



    #endif
}

//
//   Receive a packet of data from the CAN channel.
//
bool CCanChannel::ReceiveMsg(CCanMsg* pPacket, TPCANMsg *pTpPacket ,int nMicroSecondsTimeOut)
{
    #ifdef AGV_DEBUG
        return TRUE;
    #else
     TPCANRdMsg oTPCANMsgTemp;
//      int result =  RSP_CAN_RecvTimeOut( m_Canhandle , &oTPCANMsgTemp , nMicroSecondsTimeOut);
     int result =  RSP_CAN_RecvBlock( m_Canhandle , &oTPCANMsgTemp );
      if (result != RSP_RT_SUCCESS)
      {
            printf("CAN %d Recv Failed\n", m_iChannel);
            return false;
      }
      else
      {
            pPacket->id = oTPCANMsgTemp.Msg.ID;
            pPacket->rtr =oTPCANMsgTemp.Msg.MSGTYPE ;
            pPacket->dlen =oTPCANMsgTemp.Msg.LEN ;
            memcpy(pPacket->m_uchData,oTPCANMsgTemp.Msg.DATA,8);
            if(pTpPacket)
            {
                pTpPacket->ID= oTPCANMsgTemp.Msg.ID;
                pTpPacket->LEN = oTPCANMsgTemp.Msg.LEN;
                pTpPacket->MSGTYPE = oTPCANMsgTemp.Msg.MSGTYPE;
                memcpy(pTpPacket->DATA,oTPCANMsgTemp.Msg.DATA,8);
            }
            #ifdef DEBUG_OUTPUT
           // if(pPacket->id == 0x146 ||pPacket->id==0x46)
            printf_frame(pPacket->id & CAN_EFF_MASK, pPacket->m_uchData, pPacket->dlen,
                        ((pPacket->id & CAN_EFF_FLAG) ? true : false),
                        true , false);
            #endif

            return true;
      }
    #endif
}

//
//   Enable/disable receive
//
void CCanChannel::EnableReceive(bool bYes)
{

}


unsigned char CCanChannel::ReadRegister(unsigned char uchOffset)
{

        return -1;
}

////
////   Write to SJA1000 internal register.
////
void CCanChannel::WriteRegister(unsigned char uchOffset, unsigned char uchData)
{

}

////
////   Read SJA1000 internal error count.
////
unsigned long CCanChannel::ReadErrorCount(unsigned char uchType)
{
    return 0;
}

////
////   Get the version of can driver
////
int CCanChannel::GetVersion()
{
        return -1;
}

////
////   Read the NVRAM.
////
//USHORT CCanChannel::ReadNVRAM(DWORD dwAddr)
//{
//    USHORT uResult = -1;
//    return uResult;
//}

////
////   Write to the NVRAM.
////
//void CCanChannel::WriteNVRAM(DWORD dwAddr, USHORT val)
//{
//}
//
//   Get the number of packets received
//
int CCanChannel::GetRxCount()
{
      return -1;
}

//
//   Get the number of packets received
//
int CCanChannel::GetTxCount()
{
    return -1;
}

// 每调用一次本函数，帧序号自加1
static unsigned int   pframeno = 0 ;
void CCanChannel::printf_frame(const unsigned int frame_id, const unsigned char *data, const unsigned char len,
    const bool extended,
    const bool ok_flag,
    const bool sendflag)
{
    return;
    int i = 0, l = 0;
    char buf[128] = {0};
    char timestamp[128] = {0};
    struct timeval tv;
    unsigned int no = pframeno;

    l = 0;
    for (i = 0; data && i < len && i < 8; i++)
    {
        l += sprintf(buf + l, "%02X ", data[i]);
    }

    /* 时间字符串 */
    l = 0;
    (void)gettimeofday(&tv, NULL);
    (void)strftime(timestamp, 128, "%X", localtime(&tv.tv_sec));
    l = strlen(timestamp);
    (void)sprintf(timestamp + l, ".%03ld", tv.tv_usec/1000);

    printf(   "  CAN%d %s | %7u  |  %s  |",
        m_iChannel,
        sendflag ? "==>" : "<==",
        no,
        timestamp);

    {
    printf(   "  %s  | 0x%08X |  %s  |  %u  |  %s\n",
        ok_flag ? (sendflag ? "Send OK" : "Recv OK") : (sendflag ? "Send Failed" : "Recv Failed"),
        frame_id,
        extended ? "Extended frame" : "Standard frame",
        len,
        buf);
    }

    no++;
    pframeno = no;
}


#else  //Mrc  And  Gsrd
#include "Debug.h"
#include"BlackBox.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <iostream>

#include"linux/can/error.h"


#define errout(_s)    fprintf(stderr, "error class: %s\n", (_s))
#define errcode(_d) fprintf(stderr, "error code: %02x\n", (_d))

void CCanChannel::handle_err_frame(const struct can_frame *fr)
{
    if (fr->can_id & CAN_ERR_TX_TIMEOUT) {
        //std::cout<<"m_iChannel: "<<m_iChannel<<"  fr->can_id:"<<fr->can_id<<std::endl;
        printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_TX_TIMEOUT");
    }
    if (fr->can_id & CAN_ERR_LOSTARB) {
        printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_LOSTARB");
        errcode(fr->data[0]);
    }
    if (fr->can_id & CAN_ERR_CRTL) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_CRTL");
        errcode(fr->data[1]);
    }
    if (fr->can_id & CAN_ERR_PROT) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_PROT");
        errcode(fr->data[2]);
        errcode(fr->data[3]);
    }
    if (fr->can_id & CAN_ERR_TRX) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_TRX");
        errcode(fr->data[4]);
    }
    if (fr->can_id & CAN_ERR_ACK) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_ACK");
    }
    if (fr->can_id & CAN_ERR_BUSOFF) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_BUSOFF");
    }
    if (fr->can_id & CAN_ERR_BUSERROR) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_BUSERROR");
    }
    if (fr->can_id & CAN_ERR_RESTARTED) {
         printf("m_iChannel:%d ,fr->can_id: %02x \n",m_iChannel,fr->can_id);
        errout("CAN_ERR_RESTARTED");
    }
}

CCanChannel::CCanChannel()
{
    m_iChannel = 0;
    m_iSocketCanSendFd = 0 ;
    m_iSocketCanRevFd = 0;
    memset(pCanName,0,8);
}

CCanChannel::~CCanChannel()
{
    Close();
    m_iChannel = 0;
    m_iSocketCanSendFd = 0 ;
    m_iSocketCanRevFd = 0;
}

bool CCanChannel::Init(int nChannel, int nBaudRateType)
{
#ifdef AGV_DEBUG
    return true;
#else
    sprintf(pCanName, "can%d",nChannel);

    m_iChannel = nChannel;

    m_BitRate = nBaudRateType;

    SetCanConfig( nChannel, nBaudRateType , TX_QUEUE_LEN);

    m_iSocketCanSendFd = socket_connect(nChannel);

    m_iSocketCanRevFd = socket_connect(nChannel);

#ifdef _MRC_LINUX32
    {
        set_nonblocking(m_iSocketCanSendFd);
        set_nonblocking(m_iSocketCanRevFd);
    }
#endif

    if(m_iSocketCanSendFd < 0 )
    {
        perror( " m_iSocketCanSendFd < 0 " );
        return false;
    }

    if( m_iSocketCanRevFd < 0)
    {
        perror( " m_iSocketCanRevFd < 0 " );
        return false;
    }

    set_can_filter(m_iSocketCanSendFd);

    set_can_loopback(m_iSocketCanSendFd,false);

    return true;
#endif
}
bool CCanChannel::Close()
{
    #ifdef AGV_DEBUG
        return  true;
    #else
        disconnect(&m_iSocketCanSendFd);

        disconnect(&m_iSocketCanRevFd);

        CloseCanDev();

         return  true;
    #endif
}
#ifdef _MRC_LINUX32
void CCanChannel::SetCanConfig(const int port, const int bitrate,const int iSendRegisterSize)
{
    CloseCanDev();

    char	l_c8Command[128] = {0};

    memset(l_c8Command, 0, sizeof(l_c8Command));
    sprintf(l_c8Command, " ip link set can%d type can restart-ms 1",port);
    system(l_c8Command);

    if(bitrate == CAN_BR_125K)
    {
        sprintf(l_c8Command, "ip link set can%d up type can tq 250 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1", port);
        system(l_c8Command);
    }
    else if(bitrate == CAN_BR_250K)
    {
        sprintf(l_c8Command, "ip link set can%d up type can tq 125 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1", port);
        system(l_c8Command);
    }
    else if(bitrate == CAN_BR_500K)
    {
        sprintf(l_c8Command, "ip link set can%d up type can tq 75 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1", port);
        system(l_c8Command);
    }
    else
    {

    }
    //给"restart-ms"设置一个非零值可以开启总线关闭自动恢复的功能(也就是进入总线关闭状态后重新开启),下面是一个示例:
    // ip link set canX type can restart-ms 100

//    sprintf(l_c8Command, "ip -details link show can%d ", port);
//    system(l_c8Command);

//    sprintf(l_c8Command, "ip -details -statistics link show can%d ", port);
//    system(l_c8Command);

    // 设置tx_queue_len
    memset(l_c8Command, 0, sizeof(l_c8Command));
    sprintf(l_c8Command, "echo %d > /sys/class/net/can%d/tx_queue_len", iSendRegisterSize, port);
    system(l_c8Command);

    OpenCanDev();

}
#else  //_Borax_LINUX32 And Ti5728
    void CCanChannel::SetCanConfig(const int port, const int bitrate,const int iSendRegisterSize)
    {

        can_do_stop(pCanName);

        can_set_bitrate(pCanName,bitrate);

        char l_c8Command[128] = {0};

        sprintf(l_c8Command, "ip link set can%d type can restart-ms 1", port);
        system(l_c8Command);

        // 设置tx_queue_len
        memset(l_c8Command, 0, sizeof(l_c8Command));
        sprintf(l_c8Command, "echo %d > /sys/class/net/can%d/tx_queue_len", iSendRegisterSize, port);
        system(l_c8Command);

        can_do_start(pCanName);

        sprintf(l_c8Command, "ip -details link show can%d ", port);
        system(l_c8Command);
    }
#endif //

void CCanChannel::CloseCanDev()
{
    char	l_c8Command[64] = {0};

    sprintf(l_c8Command, "ifconfig can%d down", m_iChannel);

    system(l_c8Command);

}
void CCanChannel::OpenCanDev()
{
    char	l_c8Command[64] = {0};

    sprintf(l_c8Command, "ifconfig can%d up", m_iChannel);

    system(l_c8Command);
}
// 每调用一次本函数，帧序号自加1
static unsigned int   pframeno = 0 ;
void CCanChannel::printf_frame(const unsigned int frame_id, const unsigned char *data, const unsigned char len,
    const bool extended,
    const bool ok_flag,
    const bool sendflag)
{
    if(frame_id != 0x02)
    {
        return ;
    }
    int i = 0, l = 0;
    char buf[128] = {0};
    char timestamp[128] = {0};
    struct timeval tv;
    unsigned int no = pframeno;

    l = 0;
    for (i = 0; data && i < len && i < 8; i++)
    {
        l += sprintf(buf + l, "%02X ", data[i]);
    }

    /* 时间字符串 */
    l = 0;
    (void)gettimeofday(&tv, NULL);
    (void)strftime(timestamp, 128, "%X", localtime(&tv.tv_sec));
    l = strlen(timestamp);
    (void)sprintf(timestamp + l, ".%03ld", tv.tv_usec/1000);

    printf(   "  CAN%d %s | %7u  |  %s  |",
        m_iChannel,
        sendflag ? "==>" : "<==",
        no,
        timestamp);

    {
    printf(   "  %s  | 0x%08X |  %s  |  %u  |  %s\n",
        ok_flag ? (sendflag ? "Send OK" : "Recv OK") : (sendflag ? "Send Failed" : "Recv Failed"),
        frame_id,
        extended ? "Extended frame" : "Standard frame",
        len,
        buf);
    }

    no++;
    pframeno = no;
}

bool CCanChannel::SocketCanSendMsg(can_frame &pPacket)
{

   int iRet =  send_frame( m_iSocketCanSendFd, (unsigned char *)&pPacket, sizeof(can_frame));

#ifndef DEBUG_OUTPUT
   if(iRet > 0)
   printf_frame(pPacket.can_id & CAN_EFF_MASK, pPacket.data, pPacket.can_dlc,
               ((pPacket.can_id & CAN_EFF_FLAG) ? true : false),
               iRet > 0 ? true : false,
               true);
#endif
   return  iRet > 0 ? true : false;
}

bool CCanChannel::SocketCanReceiveMsg(can_frame *pPacket)
{
    memset(pPacket, 0x00, sizeof(can_frame));

    unsigned char *precvframe = (unsigned char *)pPacket;

    int  ret = recv_frame(m_iSocketCanRevFd, precvframe, sizeof(can_frame), 0);

//    handle_err_frame(pPacket);

#ifndef DEBUG_OUTPUT
    if(ret>0)
    printf_frame(pPacket->can_id & CAN_EFF_MASK, pPacket->data, pPacket->can_dlc,
                ((pPacket->can_id & CAN_EFF_FLAG) ? true : false),
                true , false);
#endif

    return  ret > 0 ? true : false;
}

//
//   Send a packet of data through the CAN channel.
//
bool CCanChannel::SendMsg(const CCanMsg *pPacket)
{
    #ifdef AGV_DEBUG
        return true;
    #else
        m_Packet = *pPacket;

        can_frame SocketCanFrame;

        CCanMsgToSCanFrameFun(*pPacket,SocketCanFrame);

        m_CritSection.Lock();
        bool iRet = SocketCanSendMsg(SocketCanFrame);
        m_CritSection.Unlock();

        return iRet;

    #endif
}

//
//   Receive a packet of data from the CAN channel.
//
bool CCanChannel::ReceiveMsg(CCanMsg* pPacket)
{
    #ifdef AGV_DEBUG
        return true;
    #else

    can_frame SocketCanFrame;

    bool iRet = SocketCanReceiveMsg(&SocketCanFrame);

    SCanFrameToCCanMsgFun(SocketCanFrame, *pPacket);

    // Callback function for customization
//    if (iRet)
//        OnReceiveMsg(pPacket);

    return iRet;

    #endif
}


int CCanChannel::set_can_loopback(const int sockfd, const bool lp)
{
    // 在默认情况下，本地回环功能是开启的，可以使用下面的方法关闭回环/开启功能：
    int loopback = lp;  // 0表示关闭, 1表示开启(默认)
    (void)setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

#if 0
    // 在本地回环功能开启的情况下，所有的发送帧都会被回环到与CAN总线接口对应的套接字上。
    // 默认情况下，发送CAN报文的套接字不想接收自己发送的报文，因此发送套接字上的回环功能是关闭的。
    // 可以在需要的时候改变这一默认行为：
    int ro = 1; // 0表示关闭(默认), 1表示开启
    (void)setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &ro, sizeof(ro));
#endif

    return  0;
}
//#define DEBUG_OUTPUT

int CCanChannel::socket_connect(const int port)
{
    return  socket_listen(port);
}

void CCanChannel::disconnect(int *sockfd)
{
    if (!sockfd || *sockfd == -1)
    {
        return ;
    }

    close_socket(*sockfd);
    *sockfd = -1;
}

int CCanChannel::send_frame(const int sockfd, const unsigned char* data, const int count)
{
    int ret = write(sockfd, data, count);
//  int ret = send(sockfd, data, count,0);
    if(ret < 0)
    {
        std::cout<<"send_frame "<<errno<<std::endl;
    }
    return  ret;
}

int CCanChannel::recv_frame(const int sockfd, unsigned char* buf, const int count, const int timeout_ms)
{
    int ret = read(sockfd, buf, count);
    //int ret = recv(sockfd, (char*)buf, count,0);

    if (ret <= 0)
    {
        return  -1;
    }

    return  ret;
}

void CCanChannel::close_socket(const int sockfd)
{
    if (sockfd != -1)
    {
        close(sockfd);
    }
}

// 绑定sock，然后监听端口
// 返回监听 套接字 文件描述符
int CCanChannel::socket_listen(const int port)
{
    int sockfd = -1;

    struct sockaddr_can _addr;
    struct ifreq _ifreq;
    char buf[256];
    int ret = 0;

     /* 建立套接字，设置为原始套接字，原始CAN协议 */
    #ifdef _MRC_LINUX32
        sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    #else  //_Borax_LINUX32  and Ti5728
    //#ifdef _Borax_LINUX32
        sockfd = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    #endif

    if (sockfd < 0)
    {
        sprintf(buf, "\n\t Open socket can%d Error\n\n", port + 1);
        panic(buf);
        return  -1;
    }

     /* 以下是对CAN接口进行初始化，如设置CAN接口名，即当我们用ifconfig命令时显示的名字 */
    sprintf(buf, "can%d", port);
    strcpy(_ifreq.ifr_name, buf);
    ret = ioctl(sockfd, SIOCGIFINDEX, &_ifreq);
    if (ret < 0)
    {
        sprintf(buf, "\n\t match socket can%d error\n\n", port + 1);
        panic(buf);
        return  -1;
    }

    /* 设置CAN协议 */
    _addr.can_family = AF_CAN;
    _addr.can_ifindex = _ifreq.ifr_ifindex;


    /* disable default receive filter on this RAW socket */
    /* This is obsolete as we do not read from the socket at all, but for */
    /* this reason we can remove the receive list in the Kernel to save a */
    /* little (really a very little!) CPU usage.                          */
    //	setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    /* 将刚生成的套接字与CAN套接字地址进行绑定 */
    ret = bind(sockfd, (struct sockaddr *)&_addr, sizeof(_addr));
    if ( ret < 0)
    {
        close_socket(sockfd);
        sprintf(buf, "\n\t bind socket can%d error\n\n", port + 1);
        panic(buf);
        return  -1;
    }

    return  sockfd;
}


int CCanChannel::set_can_filter(const int iSockFd)
{
/**
 * struct can_filter - CAN ID based filter in can_register().
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *          <received_can_id> & mask == can_id & mask
 *
 */
    const int n = 1;
    struct can_filter rfilter[n];

    // 过滤规则：当<received_can_id> & mask == can_id & mask时，接收报文，否则过滤掉.
    // 可以同时添加多条过滤规则

    // 在用来发送CAN帧的CAN_RAW套接字上不接收任何CAN帧
    rfilter[0].can_id   = 0x00000000;
    rfilter[0].can_mask = CAN_EFF_MASK;
    (void)setsockopt(iSockFd, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, n * sizeof(struct can_filter));

    // 在用来接收CAN帧的CAN_RAW套接字上禁用接收过滤规则
    //(void)setsockopt(recv_socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return  0;
}

int CCanChannel::SetSocketSndBufSize(int &iSockFd , int iSize)
{
    int opt = 0;
    socklen_t optlen = sizeof(int);
    getsockopt(iSockFd,SOL_SOCKET,SO_SNDBUF,&opt,&optlen);
    opt = SO_REUSEADDR;
    setsockopt(iSockFd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
    opt = iSize;//512k
    setsockopt(iSockFd,SOL_SOCKET,SO_SNDBUF,&opt,sizeof(int));
    getsockopt(iSockFd,SOL_SOCKET,SO_SNDBUF,&opt,&optlen);

    return 0;
}

void CCanChannel::panic(const char *msg)
{
    printf("%s",msg);
}

void CCanChannel::printf_head()
{
    printf("\n");

    printf("CAN NUM ：CAN%d        Baud rate ：%d Kbps\n\n", m_iChannel, m_BitRate / 1000);

    // CAN1 ==> |  000001  |  18:09:00.356  |  发送成功  | 0x1801F456 |  扩展帧  |  8  |  00 01 FF FF FF FF FF FF

    printf("+----------+----------+----------------+------------+------------+----------+-----+-------------------------------------------------+\n");
    printf("|   Interface   |   Num   |  Frame interval time /ms  |    Name    |    Frame ID    |  Frame Format  | DLC |  Data                   |\n");
    printf("+----------+----------+----------------+------------+------------+----------+-----+---------------------------------------------------+\n");

}

// 设置一个文件描述符为nonblock
int CCanChannel::set_nonblocking(int &fd)
{
    int flags;
    if ((flags = fcntl(fd, F_GETFL, 0)) == -1)
        flags = 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}


void CCanChannel::CCanMsgToSCanFrameFun(const CCanMsg &AgvCanMsg, can_frame &SocketCanFrame)
{

//    if(AgvCanMsg.id >  CAN_SFF_MASK) //Extend Frame
//    {
//        SocketCanFrame.can_id =  (AgvCanMsg.id & CAN_EFF_MASK);// | CAN_EFF_FLAG;
//    }
//    else  //Stand frame
//    {
//        SocketCanFrame.can_id =  (AgvCanMsg.id & CAN_SFF_MASK);
//    }

    memset(&SocketCanFrame,0,sizeof(can_frame));

     SocketCanFrame.can_id =  AgvCanMsg.id;

    if(AgvCanMsg.rtr == 0x01) // Remote Frame
    {
        SocketCanFrame.can_id =  ( SocketCanFrame.can_id | CAN_RTR_FLAG);
    }
    else
    {
//       SocketCanFrame.can_id = (SocketCanFrame.can_id & CAN_EFF_MASK);
        SocketCanFrame.can_id = (SocketCanFrame.can_id & CAN_SFF_MASK);
    }

    SocketCanFrame.can_dlc = AgvCanMsg.dlen;

    memcpy(SocketCanFrame.data, AgvCanMsg.m_uchData,  SocketCanFrame.can_dlc);

}

void CCanChannel::SCanFrameToCCanMsgFun(const can_frame &SocketCanFrame, CCanMsg &AgvCanMsg)
{
//    int exide = (SocketCanFrame.can_id & CAN_EFF_MASK) ? 1:0;
//    if(exide)
//    {

//    }
//    AgvCanMsg.id = SocketCanFrame.can_id & CAN_EFF_MASK;
    memset(&AgvCanMsg,0,sizeof(CCanMsg));

    AgvCanMsg.id = SocketCanFrame.can_id & CAN_SFF_MASK;

    AgvCanMsg.rtr = ( (SocketCanFrame.can_id & CAN_RTR_FLAG)) ? 1:0;

    AgvCanMsg.dlen = SocketCanFrame.can_dlc;

    memcpy(AgvCanMsg.m_uchData , SocketCanFrame.data,   SocketCanFrame.can_dlc);
}

int CCanChannel::GetRxCount()
{
#ifdef AGV_LINUX_DEBUG
  return 0;
#else
    struct timeval tv_timeout;
    tv_timeout.tv_sec  = 0;
    tv_timeout.tv_usec = 1;
    fd_set fs_read;
    int ret;
    FD_ZERO(&fs_read);
    FD_SET(m_iSocketCanRevFd, &fs_read);	//if fd == -1, FD_SET will block here
    //>0：就绪描述字的正数目  -1：出错  0 ：超时
    ret = select((int)m_iSocketCanRevFd + 1, &fs_read, NULL, NULL, &tv_timeout);
    return ret;
#endif

}
int CCanChannel::GetTxCount()
{
    return 0;
}

//
//   Enable/disable receive
//
void CCanChannel::EnableReceive(bool bYes)
{

}


UCHAR CCanChannel::ReadRegister(UCHAR uchOffset)
{

        return -1;
}

//
//   Write to SJA1000 internal register.
//
void CCanChannel::WriteRegister(UCHAR uchOffset, UCHAR uchData)
{

}

//
//   Read SJA1000 internal error count.
//
DWORD CCanChannel::ReadErrorCount(UCHAR uchType)
{
    return -1;
}

//
//   Get the version of can driver
//
int CCanChannel::GetVersion()
{
        return -1;
}

//
//   Read the NVRAM.
//
USHORT CCanChannel::ReadNVRAM(DWORD dwAddr)
{
    USHORT uResult = -1;
    return uResult;
}

//
//   Write to the NVRAM.
//
void CCanChannel::WriteNVRAM(DWORD dwAddr, USHORT val)
{
}

#endif //_X86_LINUX64
