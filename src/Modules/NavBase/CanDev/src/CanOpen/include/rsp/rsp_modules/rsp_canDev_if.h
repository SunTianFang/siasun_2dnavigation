/*<FH+>************************************************************************/
/*                                                                            */
/* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         */
/*                                                                            */
/*                                                                            */
/*  文件名称:rsp_can_if.h                                                              */
/*  内容摘要:                                                                 */
/*  其它说明:                                                                 */
/*  当前版本:                                                                 */
/*  作    者:QianYizhou                                                       */
/*  完成日期:2013年09月17日                                                   */
/*  修改记录:                                                                 */
/*    修改日期          版本号        修改人        修改内容                  */
/* -------------------------------------------------------------------------- */
/*    2013年09月18日          V1.0        QianYizhou        创建文件          */
/*<FH->************************************************************************/

#ifndef _RSP_CAN_IF_H
#define _RSP_CAN_IF_H

#include "rsp_define.h" 

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

//资源句柄标识
#if defined(DWORD) || defined(WORD) || defined(BYTE)
#error "double define for DWORD, WORD, BYTE found"
#endif

#define CAN_BAUDRATE_1M         1000
#define CAN_BAUDRATE_500K       500
#define CAN_BAUDRATE_250K       250
#define CAN_BAUDRATE_100K       100
#define CAN_BAUDRATE_20K        20

#define CAN_MAX_STANDARD_ID     0x7ff
#define CAN_MAX_EXTENDED_ID     0x1fffffff

// MSGTYPE bits of element MSGTYPE in structure TPCANMsg
#define MSGTYPE_STATUS        0x80     // used to mark pending status
#define MSGTYPE_EXTENDED      0x02     // declares a extended frame
#define MSGTYPE_RTR           0x01     // marks a remote frame
#define MSGTYPE_STANDARD      0x00     // marks a standard frame

// parameter nCANMsgType
#define CAN_INIT_TYPE_EX		0x01	//Extended Frame
#define CAN_INIT_TYPE_ST		0x00	//Standart Frame

typedef unsigned int            DWORD_RSP;
typedef unsigned char           BYTE_RSP;
typedef struct 
{
  DWORD_RSP ID;              // 11/29 bit code
  BYTE_RSP  MSGTYPE;         // bits of MSGTYPE_*
  BYTE_RSP  LEN;             // count of data bytes (0..8)
  BYTE_RSP  DATA[8];         // data bytes, up to 8
} TPCANMsg;              // for PCAN_WRITE_MSG

typedef struct
{
  TPCANMsg Msg;          // the above message
  DWORD_RSP    dwTime;       // a timestamp in msec, read only
} TPCANRdMsg;            // for PCAN_READ_MSG

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_Open
* 功能描述:     打开CAN设备                                                             
* 输入参数: 
*               unsigned char ucChNum   CAN设备端口（从1开始）
* 输出参数: 
*               RSP_HANDLE *pHandle     资源句柄
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Open( unsigned char ucChNum, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_Close
* 功能描述:     关闭CAN设备
* 输入参数: 
*               RSP_HANDLE handle       设备资源句柄
* 输出参数:     
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Close( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_Config
* 功能描述:     配置CAN设备                                                             
* 输入参数: 
*               RSP_HANDLE handle       资源句柄
*               int nBaundRate          波特率
*                   1000    -- 1M  kps 
*                   500     -- 500 Kps
*                   250     -- 250 kps
*                   100     -- 100 kps
*                   20      -- 20  kps
*               int nMsgType            数据类型
* 输出参数: 
*               NULL
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Config( RSP_HANDLE handle, int nBaundRate, int nMsgType  );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_SendBlock
* 功能描述:     CAN发送数据(阻塞式)
* 输入参数: 
*               RSP_HANDLE handle   CAN资源句柄
*               TPCANMsg *ptSendMsg CAN发送数据
* 输出参数:     
*               NULL
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_SendBlock( RSP_HANDLE handle, TPCANMsg* ptSendMsg );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_SendTimeOut
* 功能描述:     CAN发送数据(非阻塞式)
* 输入参数: 
*               RSP_HANDLE handle   CAN资源句柄
*               const TPCANMsg *ptSendMsg CAN发送数据
*               int nMicroSeconds   超时时间（单位:ms）
* 输出参数:     
*               NULL
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_SendTimeOut( RSP_HANDLE hHandle, TPCANMsg *ptSendMsg, int nMicroSeconds );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_RecvBlock
* 功能描述:     CAN接受数据（阻塞式）
* 输入参数: 
*               RSP_HANDLE handle       CAN资源句柄
* 输出参数:     
*               TPCANRdMsg *ptRecvMsg   CAN接受数据结构体
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_RecvBlock( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_CAN_RecvTimeOut
* 功能描述:     CAN接受数据（非阻塞式）
* 输入参数: 
*               RSP_HANDLE handle       CAN资源句柄
*               int nMicroSeconds       超时时间
* 输出参数:     
*               TPCANRdMsg *ptRecvMsg   CAN接受数据结构体
* 返 回 值: 
*               RSP_RT_SUCCESS    成功
*               其他              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_RecvTimeOut( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg, int nMicroSeconds );

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif//_RSP_CAN_IF_H



