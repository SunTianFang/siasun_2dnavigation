/*<FH+>***********************************************************************
*                                                                            
* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         
*                                                                            
*                                                                            
*  文件名称: com_test.c                                                   
*  内容摘要: 串口接口实现                                                            
*  其它说明:                                                             
*  当前版本: 1.1                                                            
*  作    者: qianyizhou                                                              
*  完成日期: 2013-7                                                              
*  修改记录:                                                                 
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-7              V1.1        qianyizhou      创建
*<FH->************************************************************************/

/******************************************************************************/
/*               #include（依次为标准库头文件、非标准库头文件）               */
/******************************************************************************/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rsp_uartDev.h"
/******************************************************************************/
/*                                  全局变量                                  */
/******************************************************************************/

/*
 * 波特率
 */
const int an_speedParam[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, };
const int an_speedName[] = { 115200, 57600, 38400, 19200, 9600, 4800, 2400, };

/******************************************************************************/
/* 标准接口                                                                   */
/******************************************************************************/

/******************************************************************************/
/*                                 全局宏定义                                 */
/******************************************************************************/
#define COM_READ_BLOCK_TIME_COUNT_DEF   (1) //串口Block计数默认参数


#define NORMAL_COM_DEV_LIMIT            4                   //串口号大于2的认为使用FPGA扩展串口
#define COM_DEV_BASE                    2

                                                            //即，串口3对应设备/dev/ttySiasun0
/*
 * 普通串口的设备描述
 */

#define COM_DEV_NORMAL                  "/dev/ttyS"

#define COM_DEV_FPGA                    "/dev/ttySiasun"    //FPGA虚拟串口的设备描述

#define DBG_RSP_UARTDEV
#ifdef DBG_RSP_UARTDEV
    #define DBG_PRINTF( format, ... )       printf( format, ##__VA_ARGS__ );
#else
    #define DBG_PRINTF( format, ... )       
#endif
/******************************************************************************/
/*                                 函数的实现                                 */
/******************************************************************************/
int RSP_UART_Open( unsigned int uiChNum, RSP_HANDLE *pHandle )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    if( NULL == pHandle ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }

    nRtValue = open_COMDevice( uiChNum, (int *)pHandle );
    if( RSP_RT_SUCCESS != nRtValue ) {
        DBG_PRINTF( "[%s]open com device failed.\n", __func__ );
        goto EXIT_LABLE;
    }

EXIT_LABLE:
    return nRtValue;
}/*}}}*/

int RSP_UART_Close( RSP_HANDLE handle )
{/*{{{*/
    return close_COMDevice( (int)handle );
}/*}}}*/

int RSP_UART_Config( RSP_HANDLE handle, PTCOM_CONFIG pComConfig )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    if( NULL == pComConfig ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        //goto EXIT_LABLE;
        return nRtValue;
    }

    nRtValue = set_COMCfg( (int)handle, 
            pComConfig->baudrate, 
            pComConfig->bits,
            pComConfig->event,
            pComConfig->stop,
            COM_READ_BLOCK_TIME_COUNT_DEF );
#if 0
    if( RSP_RT_SUCCESS != nRtValue ) {
        //如何设置失败，关闭串口设备
        DBG_PRINTF( "[%s]set com cfg failed.\n", __func__ );
        close_COMDevice( (int)handle );
        goto EXIT_LABLE;
    }

EXIT_LABLE:
#endif
    return nRtValue;
}/*}}}*/

int RSP_UART_SendBlock( RSP_HANDLE handle, const unsigned char *pucSendData, unsigned long len )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    int lenToSend = len;            //需要发送的数据长度
    int lenHaveSent = 0;            //已经发送的数据总长度
    //int lenSentPerTime = 0;         //每次已经发送的数据长度

    if( NULL == pucSendData ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }
        
    const unsigned char *pBuf = pucSendData;

    lenHaveSent = write( (int)handle, pBuf, lenToSend );
    if( lenHaveSent < 0 ) {                 //写入失败
        nRtValue = RSP_RT_ERROR_DEVICE_WRITE_FAILED;
        goto EXIT_LABLE;
    } 

EXIT_LABLE:
    return nRtValue;
}/*}}}*/

int RSP_UART_SendTimeOut( RSP_HANDLE handle, const unsigned char *pucSendData, unsigned long *pLen, int nMicroSeconds )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    if( NULL == pucSendData || NULL == pLen ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        return nRtValue;
    }
        
    const unsigned char *pBuf = pucSendData;
    int lenHaveSent = 0;            //已经发送的数据总长度

    lenHaveSent = write( (int)handle, pBuf, *pLen );
    if( lenHaveSent < 0 ) {                 //写入失败
        nRtValue = RSP_RT_ERROR_DEVICE_WRITE_FAILED;
        return nRtValue;
        //goto EXIT_LABLE;
    } 

    *pLen = lenHaveSent;
    return nRtValue;
}/*}}}*/

int RSP_UART_RecvBlock( RSP_HANDLE handle, unsigned char *pucRecvData, unsigned long len )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    int lenToRecv = len;                //需要接收的数据长度
    int lenHaveRecv = 0;                //已经接收的数据总长度
    int lenRecvPerTime = 0;             //每次已经接收的数据长度

    if( NULL == pucRecvData ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }
        
    unsigned char *pBuf = pucRecvData;

    do {
        lenRecvPerTime = read( (int)handle, pBuf, lenToRecv );
        if( lenRecvPerTime < 0 ) {
            //读取失败
            nRtValue = RSP_RT_ERROR_DEVICE_READ_FAILED;
            goto EXIT_LABLE;
        }
        lenHaveRecv += lenRecvPerTime;
        lenToRecv -= lenRecvPerTime;

        pBuf += lenRecvPerTime;

    //直到发送所有数据
    } while( lenHaveRecv < len );

EXIT_LABLE:
    return nRtValue;
}/*}}}*/

int RSP_UART_RecvTimeOut( RSP_HANDLE handle, unsigned char *pucRecvData, unsigned long *pLen, int nMicroSeconds )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    if( NULL == pucRecvData || NULL == pLen ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        //goto EXIT_LABLE;
        return nRtValue;
    }
        
    unsigned char *pBuf = pucRecvData;
    int lenToRecv = *pLen;                //需要接收的数据长度
    int lenHaveRecv = 0;                //已经接收的数据总长度
    int lenRecvPerTime = 0;             //每次已经接收的数据长度
    
    //超时时间以100ms为单位，不足100ms的记为100ms
    int nTimeOutLimit = (nMicroSeconds<100)?1:nMicroSeconds/100;

    //发生超时的次数
    int nTimeOutCount = 0;

    do {
        lenRecvPerTime = read( (int)handle, pBuf, lenToRecv );

        if( lenRecvPerTime < 0 ) {              //读取失败
            nRtValue = RSP_RT_ERROR_DEVICE_READ_FAILED;
            goto EXIT_LABLE;
        } else if ( 0 == lenRecvPerTime ) {     //发生超时
            ++nTimeOutCount;
            if( nTimeOutCount >= nTimeOutLimit ) {
                //超过限定的超时时间
                nRtValue = RSP_RT_ERROR_TIMEOUT;
                goto EXIT_LABLE;
            }
        }
        lenHaveRecv += lenRecvPerTime;
        lenToRecv -= lenRecvPerTime;

        pBuf += lenRecvPerTime;

    //直到发送所有数据
    } while( lenHaveRecv < *pLen );

EXIT_LABLE:
    *pLen = lenHaveRecv;
    return nRtValue;
}/*}}}*/

int open_COMDevice( unsigned int uiChNum, int *pFileDes )
{
    int nRtValue = RSP_RT_SUCCESS;
    char acDevName[FILENAME_MAX]    = { 0 };

    if( uiChNum < 1 ) {
        DBG_PRINTF( "please make sure COM channel[%d] is bigger than 0.\n", uiChNum );
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }

	if( uiChNum > NORMAL_COM_DEV_LIMIT) {
        DBG_PRINTF( "please make sure COM channel[%d] is no bigger than 4.\n", uiChNum );
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }

    /* get com device name */
    snprintf( acDevName, FILENAME_MAX, COM_DEV_NORMAL"%d", uiChNum - 1 + COM_DEV_BASE);
  
    DBG_PRINTF( "UART Name: %s\n", acDevName );
    //打开设备
    *pFileDes = open( acDevName, O_RDWR|O_NOCTTY ); //Blocking
    if( -1 == *pFileDes ) {
        DBG_PRINTF( "open COM device: %s failed.\n", acDevName );
        nRtValue = RSP_RT_ERROR_DEVICE_OPEN_FAILED;
        goto EXIT_LABLE;
    }

EXIT_LABLE:
    return nRtValue;
}

int set_COMSpeed( struct termios *pOpt, unsigned long ulBaudRate )
{
    //调用该接口的函数保证非NULL
    int nRtValue = RSP_RT_SUCCESS;
    int nSetInSpeed  = RSP_RT_SUCCESS;
    int nSetOutSpeed = RSP_RT_SUCCESS;
    int nIfFitParam = 0;
    int i;

    for( i=0; i<sizeof(an_speedParam)/sizeof(int); i++ ) {
        if( an_speedName[i] == ulBaudRate ) {
            nSetInSpeed = cfsetispeed( pOpt, an_speedParam[i] );
            nSetOutSpeed = cfsetospeed( pOpt, an_speedParam[i] );
            nIfFitParam = 1;
        }
    }

    if( 1 != nIfFitParam ) {
        DBG_PRINTF( "Invalid COM Baudrate Input.\n" );
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABLE;
    }

    if( RSP_RT_SUCCESS!=nSetInSpeed || RSP_RT_SUCCESS!=nSetOutSpeed ) {
        DBG_PRINTF( "COM set Baudrate failed.\n" );
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABLE;
    }
EXIT_LABLE:
    return nRtValue;
}

int set_COMCfg( 
        int nFileDes, 
        unsigned long ulBaudRate, 
        int nBits, 
        char cEvent, 
        int nStop, 
        unsigned long ulBlockCount )
{
    int nRtValue = RSP_RT_SUCCESS;
    struct termios Opt;

    /* get attribute */
    nRtValue = tcgetattr( nFileDes, &Opt );
    if( 0 != nRtValue ) {
        perror( "tcgetattr failed:" );
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABLE;
    }

    /* flush */
    tcflush( nFileDes, TCIOFLUSH );

    /* set Baudrate */
    nRtValue = set_COMSpeed( &Opt, ulBaudRate );
    if(  RSP_RT_SUCCESS != nRtValue ) {
        DBG_PRINTF( "COM:set com speed failed.\n" );
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABLE;
    }

    /* set data bits */
    switch( nBits ) {
        case 7:
            Opt.c_cflag &= ~CSIZE;
            Opt.c_cflag |= CS7;
            break;
        case 8:
            Opt.c_cflag &= ~CSIZE;
            Opt.c_cflag |= CS8;
            break;
        default:
            DBG_PRINTF( "COM: data bits not support.\n" );
            nRtValue = RSP_RT_ERROR_INVALID_ARG;
            goto EXIT_LABLE;
    }
    
    /* Event Test */
    switch( cEvent ) {
        case 'O':
        case 'o':
            Opt.c_cflag |= PARENB;
            Opt.c_cflag |= PARODD;
            Opt.c_iflag |= INPCK;
            break;
        case 'E':
        case 'e':
            Opt.c_cflag |= PARENB;
            Opt.c_cflag &= ~PARODD;
            Opt.c_iflag |= INPCK;
            break;
        case 'N':
        case 'n':
            Opt.c_cflag &= ~PARENB;
            Opt.c_iflag &= ~INPCK;
            break;
        case 'S':
        case 's':
            Opt.c_cflag &= ~PARENB;
            Opt.c_iflag |= INPCK;
            break;
        default:
            DBG_PRINTF( "COM: test event not support.\n" );
            nRtValue = RSP_RT_ERROR_INVALID_ARG;
            goto EXIT_LABLE;
    }

    /* set stop bit */
    switch( nStop ) {
        case 1:
            Opt.c_cflag &= ~CSTOPB;
            break;
        case 2:
            Opt.c_cflag |= CSTOPB;
            break;
        default:
            //assert( "COM: stop bit not support.\n" );
            nRtValue = RSP_RT_ERROR_INVALID_ARG;
            goto EXIT_LABLE;
    }

    /* set other param */
    Opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    Opt.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    Opt.c_oflag &= ~OPOST; 
    
    /* set read block cfg */
    Opt.c_cc[VTIME] = ulBlockCount; // BlockTime: ulBlockCount * 100ms
    Opt.c_cc[VMIN]  = 0;

    tcflush( nFileDes, TCIOFLUSH);

    /* set attribution */
    nRtValue = tcsetattr( nFileDes, TCSANOW, &Opt );      
    if ( 0 != nRtValue ) {        
        perror("tcsetattr ifd_com");  
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABLE;
    } 

EXIT_LABLE:
    return nRtValue;
}

int close_COMDevice( int nFileDes )
{
    int nRtValue = RSP_RT_SUCCESS;

    nRtValue = close( nFileDes );
    if ( nRtValue == -1 ) {                     /* 关闭失败      */
        nRtValue = RSP_RT_ERROR_DEVICE_CLOSE_FAILED;
    } 

    return nRtValue;
}



