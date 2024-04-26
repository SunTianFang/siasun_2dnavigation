/*<FH+>************************************************************************/
/*                                                                            */
/* 版权所有: Copyright (C) 沈阳新松机器人自动化股份有限公司                   */
/*                                                                            */
/* 文件名称: public.h                                                         */
/* 内容摘要: 公共定义头文件                                                   */
/* 其它说明:                                                                  */
/* 当前版本: 1.0                                                              */
/* 作    者: 钱益舟                                                           */
/* 完成日期: 2012-05-28                                                       */
/* 修改记录:                                                                  */
/*    修改日期          版本号        修改人        修改内容                  */
/* -------------------------------------------------------------------------- */
/*    2013-09-23        1.0           QianYizhou        创建文件                  */
/*<FH->************************************************************************/
#ifndef _RSP_DEFINE_H
#define _RSP_DEFINE_H

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

/******************************************************************************/
/*               #include（依次为标准库头文件、非标准库头文件）               */
/******************************************************************************/

/******************************************************************************/
/*                                 常量定义                                   */
/******************************************************************************/

/******************************************************************************/
/*                                全局宏定义                                  */
/******************************************************************************/
/*
 * 定义板卡类型
 */
//LX800 类型板卡
#define BD_TYPE_LX800           0
//E600  类型板卡
#define BD_TYPE_E600            1
//E3845 类型板卡
#define BD_TYPE_E3845           2

//无效的文件描述符
#define INVALID_FILE_DESCRIPTER (-1)                

/******************************************************************************/
/* 返回值宏定义                                                               */
/******************************************************************************/
#ifndef RSP_RETURN_VALUE
#define RSP_RETURN_VALUE

#define RSP_RT_ERROR_BASE   (-1000L)

#define RSP_RT_FAILED       (-1) 

#define RSP_RT_SUCCESS      (0) 

////////////////////////////////////////////////////////////////////////////////
//Common Error Info
#define RSP_RT_ERROR_INVALID_ARG    (RSP_RT_ERROR_BASE + 1) //参数无效

#define RSP_RT_ERROR_TIMEOUT        (RSP_RT_ERROR_BASE + 2) //超时

#define RSP_RT_ERROR_MALLOC_FAILED  (RSP_RT_ERROR_BASE + 3) //malloc失败
////////////////////////////////////////////////////////////////////////////////
#define RSP_RT_ERROR_DEVICE_BASE                (RSP_RT_ERROR_BASE - 1000)    //设备错误base

#define RSP_RT_ERROR_DEVICE_ALREADY_OPENED      (RSP_RT_ERROR_DEVICE_BASE + 3) //设备已经打开

#define RSP_RT_ERROR_DEVICE_ALREADY_CLOSED      (RSP_RT_ERROR_DEVICE_BASE + 4) //设备已经关闭

#define RSP_RT_ERROR_DEVICE_OPEN_FAILED         (RSP_RT_ERROR_DEVICE_BASE + 5) //设备打开失败

#define RSP_RT_ERROR_DEVICE_CLOSE_FAILED        (RSP_RT_ERROR_DEVICE_BASE + 6) //设备关闭失败

#define RSP_RT_ERROR_DEVICE_WRITE_FAILED        (RSP_RT_ERROR_DEVICE_BASE + 7) //设备写失败

#define RSP_RT_ERROR_DEVICE_READ_FAILED         (RSP_RT_ERROR_DEVICE_BASE + 8) //设备读失败

#define RSP_RT_ERROR_DEVICE_ASSIGN_RES_FAILED   (RSP_RT_ERROR_DEVICE_BASE + 9 )//设备资源分配失败

////////////////////////////////////////////////////////////////////////////////
#define RSP_RT_ERROR_POSIXTIMER_BASE            (RSP_RT_ERROR_BASE - 2000)

#define RSP_RT_ERROR_POSIX_FUNC                 (RSP_RT_ERROR_POSIXTIMER_BASE +1)

////////////////////////////////////////////////////////////////////////////////
#define RSP_RT_ERROR_SCHD_BASE                  (RSP_RT_ERROR_BASE - 3000)

#define RSP_RT_ERROR_SCHD_PRIORITY_INCOMPATIBLE (RSP_RT_ERROR_SCHD_BASE + 1)

#define RSP_RT_ERROR_SCHD_ALREADY_RUNNING       (RSP_RT_ERROR_SCHD_BASE + 2)

#define RSP_RT_ERROR_SCHD_TASK_INIT_FAILED      (RSP_RT_ERROR_SCHD_BASE + 3)

#define RSP_RT_ERROR_SCHD_NOT_INIT              (RSP_RT_ERROR_SCHD_BASE + 4)

////////////////////////////////////////////////////////////////////////////////
#define RSP_RT_ERROR_THREAD_BASE                (RSP_RT_ERROR_BASE - 4000)

#define RSP_RT_ERROR_THREAD_ATTR_SET_FAILED     (RSP_RT_ERROR_THREAD_BASE + 1)

#define RSP_RT_ERROR_THREAD_CREATE_FAILED       (RSP_RT_ERROR_THREAD_BASE + 2)

#define RSP_RT_ERROR_THREAD_SET_AFFINITY_FAILED (RSP_RT_ERROR_THREAD_BASE + 3 )

#define RSP_RT_ERROR_THREAD_GET_AFFINITY_FAILED (RSP_RT_ERROR_THREAD_BASE + 4 )

#endif//GPI_RETURN_VALUE
/******************************************************************************/
/*                              全局数据类型声明                              */
/******************************************************************************/
//typedef void *RSP_HANDLE;
#include <stdint.h>
typedef intptr_t    RSP_HANDLE;

//typedef unsigned int DWORD;
//typedef unsigned char BYTE;

/******************************************************************************/
/*                                全局变量声明                                */
/******************************************************************************/

/******************************************************************************/
/*                                外部引用定义                                */
/******************************************************************************/

/******************************************************************************/
/*                                全局函数原型                                */
/******************************************************************************/

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: _RSP_DEFINE_H */
