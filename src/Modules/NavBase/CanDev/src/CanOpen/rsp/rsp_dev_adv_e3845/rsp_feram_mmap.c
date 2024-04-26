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
//include for open/close and other signature
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
//#include "utils.h"

//include for ioctl
#include <sys/ioctl.h>

#include <sys/mman.h>

#include <stdio.h>

#include "rsp_feram.h"
#include "mapping_file_opt.h"
/******************************************************************************/
/*                                  全局变量                                  */
/******************************************************************************/

/******************************************************************************/
/* 标准接口                                                                   */
/******************************************************************************/

/******************************************************************************/
/*                                 全局宏定义                                 */
/******************************************************************************/

#define NVRAM_SIZE          (512*1024)
#define MAPPING_LEN         NVRAM_SIZE
//#define     MAPPING_LEN     (4096)

/******************************************************************************/
/*                              全局数据类型声明                              */
/******************************************************************************/

/******************************************************************************/
/*                                全局变量声明                                */
/******************************************************************************/
//时钟中断设备描述符
//#define FERAM_DEVICE            "/dev/siasun_tiedian"
#define FERAM_DEVICE            "/media/cf/.FERAM_FILE"
#define FERAM_DEVICE_ARM        "/media/cf/.FERAM_FILE_ARM"
#define FERAM_DEVICE_WHEEL      "/media/cf/.FERAM_FILE_WHELL"



//static int s_g_FeramDes = INVALID_FILE_DESCRIPTER;
//static void *s_g_pNvram = NULL;

/******************************************************************************/
/*                                 函数的实现                                 */
/******************************************************************************/
static T_CMP_FILE_DEVICE cmpFileDev;

static void *l_cmpMemAddr = NULL;

void *_new_cmp_mem( int32_t size )
{
    return malloc( size );
}

void _delete_cmp_mem( void **addr )
{
    if ( NULL != *addr ) {
        free( *addr );
        *addr = NULL;
    }
}

int RSP_Feram_Open( void** pAddr )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    
    if( NULL == pAddr ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    /*
     * init cmp mem
     */
    l_cmpMemAddr = _new_cmp_mem( MAPPING_LEN );
    if ( NULL == l_cmpMemAddr ) {
        nRtValue = RSP_RT_ERROR_MALLOC_FAILED;
        goto EXIT_LABEL;
    } else {
        printf( "[%s]PASS_LINE: %d, l_cmpMemAddr: %p\n"
                , __func__, __LINE__, l_cmpMemAddr );
    }

    /*
     * map
     */
    cmpFileDev.cmpFileName = FERAM_DEVICE;
    cmpFileDev.len = MAPPING_LEN;
    printf( "get mmap file: %s.\n", cmpFileDev.cmpFileName );

    nRtValue = initCmpFile( &cmpFileDev );
    if( FILECMP_SUCCESS != nRtValue ) {
        perror( "init cmp file failed " );
        nRtValue = RSP_RT_FAILED;
        _delete_cmp_mem( &l_cmpMemAddr );
        goto EXIT_LABEL;
    }

    *pAddr = l_cmpMemAddr;

    /*
     * feram -> cmp_mem
     */
    memcpy( l_cmpMemAddr, cmpFileDev.pMappingAddr, MAPPING_LEN );

EXIT_LABEL:
    return nRtValue;
}/*}}}*/

/*add by zhangq for feram support arm and wheel 2017.11.16*/
int RSP_Feram_Open_Extern( void** pAddr, enFeramBlock_t enBlockId)
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    
    if( NULL == pAddr ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    /*
     * init cmp mem
     */
    l_cmpMemAddr = _new_cmp_mem( MAPPING_LEN );
    if ( NULL == l_cmpMemAddr ) {
        nRtValue = RSP_RT_ERROR_MALLOC_FAILED;
        goto EXIT_LABEL;
    } else {
        printf( "[%s]PASS_LINE: %d, l_cmpMemAddr: %p\n"
                , __func__, __LINE__, l_cmpMemAddr );
    }

    /*
     * map
     */
    if (ENUM_FERAM_FOR_ARM == enBlockId)
    {
    	cmpFileDev.cmpFileName = FERAM_DEVICE_ARM;
    }
	else if (ENUM_FERAM_FOR_WHEEL == enBlockId)
	{
		cmpFileDev.cmpFileName = FERAM_DEVICE_WHEEL;
	}
	else
	{
		printf("\r\n unknown block id %d", enBlockId);
		cmpFileDev.cmpFileName = FERAM_DEVICE;
	}
		
    cmpFileDev.len = MAPPING_LEN;
    printf( "get mmap file: %s.\n", cmpFileDev.cmpFileName );

    nRtValue = initCmpFile( &cmpFileDev );
    if( FILECMP_SUCCESS != nRtValue ) {
        perror( "init cmp file failed " );
        nRtValue = RSP_RT_FAILED;
        _delete_cmp_mem( &l_cmpMemAddr );
        goto EXIT_LABEL;
    }

    *pAddr = l_cmpMemAddr;

    /*
     * feram -> cmp_mem
     */
    memcpy( l_cmpMemAddr, cmpFileDev.pMappingAddr, MAPPING_LEN );

EXIT_LABEL:
    return nRtValue;
}/*}}}*/

int RSP_Feram_Close( void )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;

    /*
     * close
     */
    nRtValue = uninitCmpFile( &cmpFileDev );
    if( FILECMP_SUCCESS != nRtValue ) {
        perror( "sync cmp file failed " );
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABEL;
    }

EXIT_LABEL:
    _delete_cmp_mem( &l_cmpMemAddr );
    return nRtValue;
}/*}}}*/

int RSP_Feram_Read( unsigned int pos, unsigned char *pBuf, unsigned int len )
{/*{{{*/
    int nRtValue = RSP_RT_SUCCESS;
    unsigned long ulTotalLen = 0;

    /* 
     * 入参校验 
     */
    if( pos >= NVRAM_SIZE ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    if( NULL == pBuf ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    ulTotalLen = pos + len;
    if( ulTotalLen > NVRAM_SIZE ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    /* 由铁电直接复制 */
    //memcpy( pBuf, cmpFileDev.pMappingAddr + pos, len );
    memcpy( pBuf, l_cmpMemAddr + pos, len );

EXIT_LABEL:
    return nRtValue;
}/*}}}*/

int RSP_Feram_Write( unsigned int pos, const unsigned char *pBuf, unsigned int len )
{
    int nRtValue = RSP_RT_SUCCESS;
    unsigned long ulTotalLen = 0;

    /* 
     * 入参校验 
     */
    if( pos >= NVRAM_SIZE ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    if( NULL == pBuf ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    ulTotalLen = pos + len;
    if( ulTotalLen > NVRAM_SIZE ) {
        nRtValue = RSP_RT_ERROR_INVALID_ARG;
        goto EXIT_LABEL;
    }

    /* 由铁电直接复制 */
    //memcpy( cmpFileDev.pMappingAddr + pos, (void *)pBuf, len );
    memcpy( l_cmpMemAddr + pos, (void *)pBuf, len );

#if 0
    nRtValue = syncCmpFile( &cmpFileDev );
    if( FILECMP_SUCCESS != nRtValue ) {
        perror( "sync cmp file failed " );
        nRtValue = RSP_RT_FAILED;
        goto EXIT_LABEL;
    }
#endif

EXIT_LABEL:
    return nRtValue;
}

//#include <sys/time.h> 
#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000 
#endif                         
#define timerdiff(a,b) ((float)((a)->tv_sec - (b)->tv_sec) + \
                         ((float)((a)->tv_nsec - (b)->tv_nsec))/NSEC_PER_SEC)
//FIXME: opt the logic
#if 0/*{{{*/
bool RSP_Feram_Sync( void )
{/*{{{*/
    static struct timespec cur, last;
    static bool ifFirstTime = true;
    int cmpResult = 0;
    bool ifSync = false;

    //if ( true == ifFirstTime ) {
    //    clock_gettime( CLOCK_MONOTONIC, &last );
    //} else {
    //    last.tv_sec = cur.tv_sec; last.tv_nsec = cur.tv_nsec;
    //}
    clock_gettime( CLOCK_MONOTONIC, &cur );
    float timediff = timerdiff( &cur, &last );
    //printf( "[%s]timediff: %f\n", __func__, timediff );

    if ( true == ifFirstTime 
            || timediff > 1.0f ) {
        cmpResult = memcmp( l_cmpMemAddr, cmpFileDev.pMappingAddr, MAPPING_LEN );
        //printf( "[%s]cmpResult: %d\n", __func__, cmpResult );
        if ( cmpResult != 0 ) {
            memcpy( cmpFileDev.pMappingAddr, l_cmpMemAddr, MAPPING_LEN );
            syncCmpFile( &cmpFileDev );
            ifSync = true;
            printf( "[%s]call sync!\n", __func__ );
            last.tv_sec = cur.tv_sec; last.tv_nsec = cur.tv_nsec;
        }
    }

    ifFirstTime = false;

    return ifSync;
}/*}}}*/
#else/*}}}*/

bool RSP_Feram_Sync( void )
{/*{{{*/
    static struct timespec cur = {0,0};
    static struct timespec last_sync = {0,0};
    static bool ifFirstTime = true;

    static float timediff = 0.0f;

    bool ifSync = false;
    bool ifCmp  = false;
    int cmpResult = 0;

    clock_gettime( CLOCK_MONOTONIC, &cur );

    ifCmp   = false;
    if ( true == ifFirstTime ) {
        ifCmp = true;
        ifFirstTime = false;
    } else {
        timediff = timerdiff( &cur, &last_sync );
        //printf( "[%s]timediff: %f\n", __func__, timediff );
        if ( timediff > 1.0f ) {
            ifCmp = true;
        }
    }

    ifSync  = false;
    if ( true == ifCmp ) {
        cmpResult = memcmp( l_cmpMemAddr, cmpFileDev.pMappingAddr, MAPPING_LEN );
        if ( cmpResult != 0 ) {
            ifSync = true;
        }
    }

    if ( true == ifSync ) {
        memcpy( cmpFileDev.pMappingAddr, l_cmpMemAddr, MAPPING_LEN );
        syncCmpFile( &cmpFileDev );
        ifSync = true;
        //printf( "[%s]call sync!\n", __func__ );
        last_sync.tv_sec = cur.tv_sec;
        last_sync.tv_nsec = cur.tv_nsec;
    }

    //printf( "ifSync: %d, ifCmp: %d, cmp result: %d\n", ifSync, ifCmp, cmpResult );
    return ifSync;
}/*}}}*/
#endif
