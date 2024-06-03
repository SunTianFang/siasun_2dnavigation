/*<FH+>************************************************************************/
/*                                                                            */
/* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         */
/*                                                                            */
/*                                                                            */
/*  文件名称:rsp_com_if.h                                                              */
/*  内容摘要:                                                                 */
/*  其它说明:                                                                 */
/*  当前版本:                                                                 */
/*  作    者:QianYizhou                                                       */
/*  完成日期:2013年09月18日                                                   */
/*  修改记录:                                                                 */
/*    修改日期          版本号        修改人        修改内容                  */
/* -------------------------------------------------------------------------- */
/*    2013年09月18日          V1.0        QianYizhou        创建文件          */
/*<FH->************************************************************************/
#ifndef _RSP_COM_IF_H
#define _RSP_COM_IF_H

#include "rsp_define.h"

#ifdef __cplusplus
extern "C"
{
#endif

//COM口参数设置结构体
typedef struct t_com_config{
    unsigned long baudrate;     //波特率
                                //  可用的波特率：
                                //  115200, 57600, 38400, 19200, 9600, 4800, 2400
    int bits;         //数据位
    int stop;         //停止位 
    char event;        //校验方式： //  'N':无校验;     //  'E':偶校验;  //  'O':奇校验;     //  'S':空校验;
}T_COM_CONFIG,
*PTCOM_CONFIG;

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_Open                                                             
* 功能描述:     打开串口设备
* 输入参数:                                                              
*               unsigned int uiChNum        串口通道号，范围：(1－...)
*                                           当通道号>2时，打开的为FPGA扩展串口
* 输出参数:     
*               RSP_HANDLE *pHandle         COM资源标识
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
int RSP_UART_Open( unsigned int uiChNum, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_Close                                                             
* 功能描述:     关闭串口设备                                                             
* 输入参数: 
*               RSP_HANDLE        COM资源标识
* 输出参数:                                                              
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
int RSP_UART_Close( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_Config                                                             
* 功能描述:     打开串口设备
* 输入参数:                                                              
*               RSP_HANDLE handle           COM资源标识
*                                           当通道号>2时，打开的为FPGA扩展串口
*               PTCOM_CONFIG pComConfig     串口参数配置
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
int RSP_UART_Config( RSP_HANDLE handle, PTCOM_CONFIG pComConfig );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_SendBlock                                                             
* 功能描述:     向串口发送数据（阻塞）
* 输入参数: 
*               RSP_HANDLE                  COM资源标识
*               unsigned char *pucSendData  发送数据缓存指针
*               unsigned long len           发送数据长度
* 输出参数:                                                              
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
int RSP_UART_SendBlock( RSP_HANDLE handle, const unsigned char *pucSendData, unsigned long len );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_SendTimeOut
* 功能描述:     向串口发送数据（非阻塞）
* 输入参数: 
*               RSP_HANDLE                          COM资源标识
*               const unsigned char *pucSendData    发送数据缓存指针
*               unsigned long *pLen                 发送数据长度
*               为何此处需要传入指针？ 当非阻塞发送时，可能部分数据已经发送完毕，但是仍有为发送完成，但此时发送超时。
*                                       因此用rtValue作为是否超时的指示，＊pLen作为成功发送数据长度的指示
*               int nMicroSeconds                   超时时间（单位:ms）
* 输出参数:
*               unsigned long *pLen                 成功发送数据长度
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
int RSP_UART_SendTimeOut( RSP_HANDLE handle, const unsigned char *pucSendData, unsigned long *pLen, int nMicroSeconds );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_RecvBlock                                                             
* 功能描述:     串口接受数据（阻塞）                                                             
* 输入参数:                                                              
*               RSP_HANDLE handle           文件描述符
*               unsigned long len           接收数据长度
* 输出参数:                                                              
*               unsigned char *pucRecvData  接收数据缓存指针
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
int RSP_UART_RecvBlock( RSP_HANDLE handle, unsigned char *pucRecvData, unsigned long len );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UART_RecvTimeOut                                                             
* 功能描述:     串口接受数据（阻塞）                                                             
* 输入参数:                                                              
*               RSP_HANDLE handle           文件描述符
*               unsigned long len           接收数据长度
* 输出参数:                                                              
*               unsigned char *pucRecvData  接收数据缓存指针
*               unsigned long *pLen         成功接收数据长度
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
int RSP_UART_RecvTimeOut( RSP_HANDLE handle, unsigned char *pucRecvData, unsigned long *pLen, int nMicroSeconds );

#ifdef  __cplusplus
}
#endif

#endif /* end of include guard: _RSP_COM_IF_H */


