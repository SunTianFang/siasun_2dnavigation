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
#include <sys/time.h> 
#include <time.h> 

#include "rsp_clockDev.h"

//include for open/close and other signature
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

//include for ioctl
#include <sys/ioctl.h>

#include "utils.h"

/******************************************************************************/
/*                                  全局变量                                  */
/******************************************************************************/

/******************************************************************************/
/* 标准接口                                                                   */
/******************************************************************************/

/******************************************************************************/
/*                                 全局宏定义                                 */
/******************************************************************************/

//时钟设备
#define CLOCK_IRQ_DEVICE         "/dev/siasun_timer"

// ioctl command for siasun_timer
#define SSS_IOC_MAGIC 'k'

#define T_SET       _IOWR(SSS_IOC_MAGIC, 0 << 6 | 0x00, int)    //设置时钟，单位：ms，范围（0－255）
#define T_ENB       _IO(SSS_IOC_MAGIC, 0 << 6 | 0x01)           //使能中断
#define T_UNE       _IO(SSS_IOC_MAGIC, 0 << 6 | 0x02)           //禁能中断

//时钟中断周期min
#define TIME_INTERVAL_MIN       (0)

//时钟中断周期max
#define TIME_INTERVAL_MAX       (255)

//无效的文件描述符
//#define INVALID_FILE_DESCRIPTER (-1)                
//defined in public.h

/******************************************************************************/
/*                              全局数据类型声明                              */
/******************************************************************************/

/******************************************************************************/
/*                                全局变量声明                                */
/******************************************************************************/
static unsigned long long s_interval_nanoSec = 0;

/******************************************************************************/
/*                                 函数的实现                                 */
/******************************************************************************/
int RSP_Clock_Open( void )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    return nRtValue;
}/*}}}*/

int RSP_Clock_Close( void )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    return nRtValue;
}/*}}}*/

int RSP_Clock_Config_ns( int ns )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    if ( ns < TIME_INTERVAL_MIN ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        return nRtValue;
    }

    s_interval_nanoSec = ns;

    return nRtValue;
}/*}}}*/

int RSP_Clock_Config( int nMilliSeconds )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    //入参判断
    if( nMilliSeconds < TIME_INTERVAL_MIN || nMilliSeconds > TIME_INTERVAL_MAX ) {
        //参数无效
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        return nRtValue;
    }

    s_interval_nanoSec = nMilliSeconds * 1000000;
    return nRtValue;
}/*}}}*/

void RSP_Clock_EnableIrq( void )
{/*{{{*/
    return;
}/*}}}*/

void RSP_Clock_DisableIrq( void )
{/*{{{*/
    return;
}/*}}}*/

static int s_ifInitClock = 0;
static struct timespec nextTime, curTime;
void RSP_Clock_WaitIrq( void )
{/*{{{*/
    if ( !s_ifInitClock ) {
        clock_gettime( CLOCK_MONOTONIC, &curTime );
        nextTime.tv_sec = curTime.tv_sec;
        nextTime.tv_nsec = curTime.tv_nsec;
        //plus time;
        timerplusnsec( &nextTime, s_interval_nanoSec );
        s_ifInitClock = 1;
    }
    Try0(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, 
                &nextTime, NULL)); 

    timerplusnsec( &nextTime, s_interval_nanoSec );

    return;
}/*}}}*/

