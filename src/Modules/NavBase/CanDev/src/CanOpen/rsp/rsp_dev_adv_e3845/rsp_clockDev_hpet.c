/*<FH+>***********************************************************************
*                                                                            
* ��Ȩ����: Copyright (C) ���������Զ����ɷ����޹�˾                         
*                                                                            
*                                                                            
*  �ļ�����: com_test.c                                                   
*  ����ժҪ: ���ڽӿ�ʵ��                                                            
*  ����˵��:                                                             
*  ��ǰ�汾: 1.1                                                            
*  ��    ��: qianyizhou                                                              
*  �������: 2013-7                                                              
*  �޸ļ�¼:                                                                 
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-7              V1.1        qianyizhou      ����
*<FH->************************************************************************/

/******************************************************************************/
/*               #include������Ϊ��׼��ͷ�ļ����Ǳ�׼��ͷ�ļ���               */
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
/*                                  ȫ�ֱ���                                  */
/******************************************************************************/

/******************************************************************************/
/* ��׼�ӿ�                                                                   */
/******************************************************************************/

/******************************************************************************/
/*                                 ȫ�ֺ궨��                                 */
/******************************************************************************/

//ʱ���豸
#define CLOCK_IRQ_DEVICE         "/dev/siasun_timer"

// ioctl command for siasun_timer
#define SSS_IOC_MAGIC 'k'

#define T_SET       _IOWR(SSS_IOC_MAGIC, 0 << 6 | 0x00, int)    //����ʱ�ӣ���λ��ms����Χ��0��255��
#define T_ENB       _IO(SSS_IOC_MAGIC, 0 << 6 | 0x01)           //ʹ���ж�
#define T_UNE       _IO(SSS_IOC_MAGIC, 0 << 6 | 0x02)           //�����ж�

//ʱ���ж�����min
#define TIME_INTERVAL_MIN       (0)

//ʱ���ж�����max
#define TIME_INTERVAL_MAX       (255)

//��Ч���ļ�������
//#define INVALID_FILE_DESCRIPTER (-1)                
//defined in public.h

/******************************************************************************/
/*                              ȫ��������������                              */
/******************************************************************************/

/******************************************************************************/
/*                                ȫ�ֱ�������                                */
/******************************************************************************/
static unsigned long long s_interval_nanoSec = 0;

/******************************************************************************/
/*                                 ������ʵ��                                 */
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

    //����ж�
    if( nMilliSeconds < TIME_INTERVAL_MIN || nMilliSeconds > TIME_INTERVAL_MAX ) {
        //������Ч
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

