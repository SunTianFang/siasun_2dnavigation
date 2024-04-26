#include <stdio.h>
//#include <math.h>
#include <string.h>
#include <fcntl.h>
//#include <signal.h>
//#include <stdio.h>
#include <unistd.h>
//#include <linux/ioctl.h>
//#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
//#include <linux/delay.h>

#include "pcan.h"

#include "libpcan.h"

#include "rsp_canDev.h"

#include "vcil_can_headers/SUSI_IMC.h"


#define CAN_DEV_DES         "/dev/pcan"     //CAN璁惧鎻忚堪

//QianYizhou Modify 
//鍘婚櫎閮ㄥ垎FPGA鏃犳硶姝ｇ‘璁剧疆鐨勬尝鐗圭巼 
//const WORD an_canSpeedParam[] = { CAN_BAUD_1M, CAN_BAUD_500K, CAN_BAUD_250K, CAN_BAUD_125K, CAN_BAUD_100K, CAN_BAUD_50K, CAN_BAUD_20K, CAN_BAUD_10K, CAN_BAUD_5K };
//const WORD an_canSpeedParam[] = { CAN_BAUD_1M, CAN_BAUD_500K, CAN_BAUD_250K, CAN_BAUD_100K, CAN_BAUD_20K };

const int an_canSpeedName[] = { 1000, 500, 250, 125, 100};
//Modify Ended 2014-4-2 by QianYizhou

/*研华E3845 CAN驱动程序*/
//typedef struct _OSEVENT {pthread_cond_t event_handle; pthread_mutex_t mutex_handle;} OSEVENT;
/*can通道 handle CAN1 = 1  CAN2 = 2*/
int siCanHandle[2] = {1, 2};

/*can接收事件变量定义*/
//static OSEVENT can_rx_event[2];

/*CAN波特率定义*/
const WORD an_canSpeedParam[] = { CAN_SPEED_1M, CAN_SPEED_500K, CAN_SPEED_250K, CAN_SPEED_125K, CAN_SPEED_100K};

#define   CAN_OPEN_TRUE                   1
#define   CAN_OPEN_FALSE                  0

/*CAN OPEN FLAG*/
static int siCanOpenFlag[2] = {CAN_OPEN_FALSE, CAN_OPEN_FALSE};

#define CAN_READ_MODE_EVENT               0
#define CAN_READ_MODE_POLL                1

/*CAN Read Mode*/
static int siCanReadMode[2] = {CAN_READ_MODE_POLL,CAN_READ_MODE_POLL};

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000 
#endif  

#define timerplusnsec(c,d) (c)->tv_nsec +=(d);             \
                           if ((c)->tv_nsec >NSEC_PER_SEC){ \
                           (c)->tv_nsec -= NSEC_PER_SEC;(c)->tv_sec++;}


//#define RSP_ECAT_DBG
#ifdef  RSP_ECAT_DBG
    #define DEBUG_PASS_LINE                     printf( "[%s]Pass LINE: %d\n", __func__, __LINE__ );
    #define DEBUG_INFO( fmt, ... )              printf( fmt, ##__VA_ARGS__ )
    #define DEBUG_INFO_DETAIL( fmt, ... )       printf( "[FUNC:%s][LINE:%04d]"fmt, __func__, __LINE__, ##__VA_ARGS__ )
    #include <assert.h>
    #define ASSERT( content )                   assert( content )
#else
    #define DEBUG_PASS_LINE
    #define DEBUG_INFO( fmt, ... )
    #define DEBUG_INFO_DETAIL( fmt, ... )
    #define ASSERT( content )                   
#endif

static int get_CANSpeed( int speedName, WORD *speedParam )
{
    int nIfFitParam = 0;
    int i;

    for( i=0; i<sizeof(an_canSpeedParam)/sizeof(WORD); i++ ) {
        if( an_canSpeedName[i] == speedName ) {
            *speedParam = an_canSpeedParam[i];
            nIfFitParam = 1;
        }
    }
    return (nIfFitParam?RSP_RT_SUCCESS:RSP_RT_ERROR_INVALID_ARG);
}

int  RSP_CAN_SDK_Init(unsigned char ucChNum)
{
	int result = 0;
	//char library_version[64]={0};

	if( (result = SUSI_IMC_VCIL_Initialize(ucChNum)) != IMC_ERR_NO_ERROR )
	{
		DEBUG_INFO("SUSI_IMC_VCIL_Initialize fail. error code=0x%04x\n", result);
		return RSP_RT_FAILED;
	}
	return RSP_RT_SUCCESS;
}
int RSP_CAN_Open( unsigned char ucChNum, RSP_HANDLE *pGPI_Handle )
{
    //int nRtValue = RSP_RT_SUCCESS;
    
	int iArrayIndex = 0;
	//int iResult = 0;
	static int siSdkInitFlag = 0;

    if ( ucChNum <= 0 || ucChNum >= 3 ) {
        return RSP_RT_ERROR_INVALID_ARG;
    }

    if( NULL == pGPI_Handle ) {
        return RSP_RT_ERROR_INVALID_ARG;
    }

	RSP_CAN_SDK_Init(ucChNum);

	/*CAN模块结构数组索引from 0*/
	iArrayIndex = ucChNum - 1;

	/*记录CAN通道号*/
	*pGPI_Handle = (intptr_t)&siCanHandle[iArrayIndex];

	/*记录CAN OPEN状态*/
	siCanOpenFlag[iArrayIndex] = CAN_OPEN_TRUE;

	/*默认采用消息触发模式读取CAN数据*/
	siCanReadMode[iArrayIndex] = CAN_READ_MODE_EVENT;
	
	return RSP_RT_SUCCESS;
}
int RSP_CAN_Close( RSP_HANDLE handle )
{/*{{{*/
	int iCanCh = 0;

	iCanCh = *(int *)handle;
	
	/*CAN open状态关闭*/
	siCanOpenFlag[iCanCh - 1] = CAN_OPEN_FALSE;
	siCanReadMode[iCanCh - 1] = CAN_READ_MODE_POLL;
	SUSI_IMC_VCIL_Deinitialize(iCanCh);
	
    return RSP_RT_SUCCESS;
}/*}}}*/

int RSP_CAN_Config( RSP_HANDLE handle, int nBaundRate, int nMsgType )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    WORD wSpeedParam;
    int iCanCh = 0;
    enum CAN_SPEED bitrate;
	int iResult = 0;

	iCanCh = *(int *)handle;   

	/* get speedParam */
    nRtValue = get_CANSpeed( nBaundRate, &wSpeedParam );
    //DEBUG_INFO( "CANSpeed: %d, %d\n", nBaundRate, wSpeedParam );
    if( RSP_RT_SUCCESS != nRtValue ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        return nRtValue;
    }

    /* TYPE */
    if( CAN_INIT_TYPE_EX != nMsgType && CAN_INIT_TYPE_ST != nMsgType ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        return nRtValue;
    }

	iCanCh = *(int *)handle;
	bitrate = wSpeedParam;
	
	if( (iResult = SUSI_IMC_CAN_SetBitTiming(iCanCh, bitrate)) != IMC_ERR_NO_ERROR )
	{
		DEBUG_INFO("SUSI_IMC_CAN_SetBitTiming fail. error code=0x%04x\n", iResult);
		return RSP_RT_FAILED;
	}
	
    return nRtValue;
}/*}}}*/

/*CAN报文类型转换 应用->SDK驱动接口*/
void can_msg_type_translate_to_driver(int iOldType, int *piNewType)
{
	switch(iOldType)
	{
		case MSGTYPE_STANDARD:
			*piNewType = CAN_MESSAGE_STANDARD;
			break;
			
		case MSGTYPE_EXTENDED:
			*piNewType = CAN_MESSAGE_EXTENDED;
			break;	

		case MSGTYPE_STANDARD_RTR:
			*piNewType = CAN_MESSAGE_STANDARD | CAN_MESSAGE_RTR;
			break;
			
		case MSGTYPE_EXTENDED_RTR:
			*piNewType = CAN_MESSAGE_EXTENDED | CAN_MESSAGE_RTR;
			break;	
			
		default:
			*piNewType = CAN_MESSAGE_STANDARD;
			DEBUG_INFO("\r\n Error can msg type %x", iOldType);
			break;
			
	}
	
}

/*CAN报文类型转换 SDK驱动接口->应用接口*/
//void can_msg_type_translate_from_driver(int iOldType, int *piNewType)
void can_msg_type_translate_from_driver(int iOldType, BYTE *piNewType)
{
	if (iOldType & CAN_MESSAGE_RTR)
	{
		if (iOldType & CAN_MESSAGE_STANDARD)
		{
			*piNewType = MSGTYPE_STANDARD_RTR;
		}
		else
		{
			*piNewType = MSGTYPE_EXTENDED_RTR;
		}
	}
	else
	{
		if (iOldType & CAN_MESSAGE_STANDARD)
		{
			*piNewType = MSGTYPE_STANDARD;
		}
		else
		{
			*piNewType = MSGTYPE_EXTENDED;
		}
	}
	
}

//int RSP_CAN_SendBlock( RSP_HANDLE handle, TPCANMsg* ptSendMsg )
int _RSP_CAN_SendBlock( RSP_HANDLE handle, TPCANMsg* ptSendMsg )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
	int iResult = 0;
	int iCanCh = 0;

	iCanCh = *(int *)handle;
	
	IMC_CAN_MSG_OBJECT message;

    memset(&message, 0, sizeof(IMC_CAN_MSG_OBJECT));
	
	message.id = ptSendMsg->ID;

	can_msg_type_translate_to_driver(ptSendMsg->MSGTYPE, &message.message_type);

	memcpy(&message.buf[0], &ptSendMsg->DATA[0], ptSendMsg->LEN);

	message.buf_len = ptSendMsg->LEN;
										
    //DEBUG_INFO( "[%s]iCanCh: %d, id: %d, buf_len: %d, Msg.type: %d, msg_type: %d\n"
    //        , __func__, iCanCh, message.id, message.buf_len, ptSendMsg ->MSGTYPE, message.message_type );
	if( (iResult = SUSI_IMC_CAN_Write(iCanCh, &message)) != IMC_ERR_NO_ERROR )
	{
		DEBUG_INFO("SUSI_IMC_CAN_Write fail. error code=0x%04x\n", iResult);
		DEBUG_INFO("\r\n ptSendMsg: ID %d, MSGTYPE %d, LEN: %d\n", ptSendMsg->ID, ptSendMsg->MSGTYPE, ptSendMsg->LEN);
        nRtValue = RSP_RT_ERROR_DEVICE_WRITE_FAILED;
	}
	
    return nRtValue;
}

/*The Can Sdk not support send time out*/
int RSP_CAN_SendTimeOut( RSP_HANDLE hHandle, TPCANMsg *ptSendMsg, int nMicroSeconds )
{ 
	return (RSP_CAN_SendBlock(hHandle, ptSendMsg));
}


/*CAN SDK阻塞接收接口*/
int RSP_CAN_RecvBlock( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg )
{
	int nRtValue = RSP_RT_SUCCESS;
	//int i = 0;	
	int iCanCh = 0;
	int iArrayIndex = 0;
	USHORT result;
	IMC_CAN_MSG_OBJECT message;

	iCanCh = *(int *)handle;
	iArrayIndex = iCanCh - 1;

	if (siCanOpenFlag[iArrayIndex])	
	{	
		//DEBUG_INFO("\r\n iCanCh %d file %s line %d\n",iCanCh, __FILE__,__LINE__);
		result = SUSI_IMC_CAN_Read(iCanCh, &message); 
		if (result == IMC_ERR_NO_ERROR)
		{		
			//DEBUG_INFO("\r\n iCanCh %d file %s line %d\n",iCanCh, __FILE__,__LINE__);
			ptRecvMsg->Msg.ID = message.id;				
			can_msg_type_translate_from_driver(message.message_type, &ptRecvMsg->Msg.MSGTYPE);
			memcpy(&ptRecvMsg->Msg.DATA[0], &message.buf[0], message.buf_len);
			ptRecvMsg->Msg.LEN = message.buf_len;
			nRtValue = RSP_RT_SUCCESS;
		}
		else
		{			
			DEBUG_INFO("Can Port %d receive failed. error code=0x%04x\n", iCanCh, (unsigned int)result);
			nRtValue = RSP_RT_ERROR_DEVICE_READ_FAILED;
		}
		
	}
    return nRtValue;
}

int RSP_CAN_SendBlock( RSP_HANDLE handle, TPCANMsg* ptSendMsg )
{
    int nRtValue = RSP_RT_SUCCESS;
    //int i = 0;
	int iReTryCount = 0;

    //DEBUG_INFO( "[%s]called in lib\n", __func__ );
    do{		
        //GPI_Clock_timestamp_start((pInfo->sendChannel - 1));
        nRtValue = _RSP_CAN_SendBlock( handle, ptSendMsg );
        iReTryCount++;
        usleep( 10 );
        //if (RSP_RT_SUCCESS != nRtValue)
        //{
        //    istErrCount[pInfo->sendChannel - 1]++;
        //    GPI_Clock_timestamp_end((pInfo->sendChannel - 1));
        //}
    }while((iReTryCount < 5) && (RSP_RT_SUCCESS != nRtValue));

    return nRtValue;
}

/*CAN SDK 超时阻塞接收接口*/

int RSP_CAN_RecvTimeOut( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg, int nMicroSeconds )
{/*{{{*/
    return RSP_CAN_RecvBlock( handle, ptRecvMsg );
#if 0
    int nRtValue = RSP_RT_SUCCESS;
	int i = 0;	
	int iCanCh = 0;
	int iArrayIndex = 0;
	USHORT result;
	IMC_CAN_MSG_OBJECT message;

	iCanCh = *(int *)handle;
	iArrayIndex = iCanCh - 1;

	if (siCanOpenFlag[iArrayIndex])	
	{	

		DEBUG_INFO("\r\n iCanCh %d file %s line %d",iCanCh, __FILE__,__LINE__);
		result = SUSI_IMC_CAN_ReadTimedWait(iCanCh, &message, (nMicroSeconds / 1000)); 
		if (result == IMC_ERR_NO_ERROR)
		{		
			DEBUG_INFO("\r\n iCanCh %d file %s line %d",iCanCh, __FILE__,__LINE__);
			ptRecvMsg->Msg.ID = message.id;				
			can_msg_type_translate_from_driver(message.message_type, &ptRecvMsg->Msg.MSGTYPE);
			memcpy(&ptRecvMsg->Msg.DATA[0], &message.buf[0], message.buf_len);
			ptRecvMsg->Msg.LEN = message.buf_len;
			nRtValue = RSP_RT_SUCCESS;
		
		}
		else
		{
			DEBUG_INFO("\r\n iCanCh %d file %s line %d",iCanCh, __FILE__,__LINE__);
			if( result == IMC_ERR_TIMEOUT )
			{
				DEBUG_INFO("Can Port %d receive timeout. error code=0x%04x\n", iCanCh, (unsigned int)result);
			    nRtValue = RSP_RT_ERROR_TIMEOUT;
			}
			else
			{
				DEBUG_INFO("Can Port %d receive failed. error code=0x%04x\n", iCanCh, (unsigned int)result);
				nRtValue = RSP_RT_ERROR_DEVICE_READ_FAILED;
			}
		}
		
	}
    return nRtValue;
#endif

}/*}}}*/

