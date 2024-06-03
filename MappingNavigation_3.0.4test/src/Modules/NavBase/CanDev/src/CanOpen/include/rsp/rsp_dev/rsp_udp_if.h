/*<FH+>************************************************************************/
/*                                                                            */
/* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         */
/*                                                                            */
/*                                                                            */
/*  文件名称:rsp_udp_server_if.h                                                              */
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
#ifndef _RSP_UDP_IF_H
#define _RSP_UDP_IF_H

#include "rsp.h"

//#include <sys/socket.h>

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_Init
* 功能描述:     UDP实例初始化
* 输入参数:
*               int serverPort          UDP端口号
* 输出参数:                                                              
*               RSP_HANDLE *pHandle     资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_Init( int port, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_Uninit
* 功能描述:     UPD实例反初始化
* 输入参数:
*               RSP_HANDLE *pHandle     资源标识
* 输出参数:                                                              
*               RSP_HANDLE *pHandle     资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_Uninit( RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_GetDstConnectHandle
* 功能描述:     UDP获取目标资源标识-connect方式
* 输入参数:
*               const char *ipAddr          目标IP地址
*               int port                    目标端口
* 输出参数:                                                              
*               RSP_HANDLE *pHandle         目标资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               对相对固定的UDP连接，应该使用RSP_UDP_GetDstConnectHandle方式获取目标资源句柄。
*               此接口将local handle与remote handle建立固定连接，数据收发效率有较大提升。
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_GetDstConnectHandle( RSP_HANDLE local, const char *ipAddr, int port, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_GetDstHandle
* 功能描述:     UDP获取目标资源标识
* 输入参数:
*               const char *ipAddr          目标IP地址
*               int port                    目标端口
* 输出参数:                                                              
*               RSP_HANDLE *pHandle         目标资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               对相对固定的UDP连接，应该使用RSP_UDP_GetDstConnectHandle方式获取目标资源句柄。
*               此接口将local handle与remote handle建立固定连接，数据收发效率有较大提升。
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_GetDstHandle( const char *ipAddr, int port, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_GetDstHandle
* 功能描述:     UDP释放标资源标识
* 输入参数:
*               RSP_HANDLE *pHandle         目标资源标识
* 输出参数:                                                              
*               RSP_HANDLE *pHandle         目标资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_ReleaseDstHandle( RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_Send
* 功能描述:     UDP发送数据
* 输入参数:
*               RSP_HANDLE local            本地UDP资源标识
*               RSP_HANDLE remote           远端UDP资源标识
*               const unsigned char *data   发送数据缓存
*               unsigned long *pLen         发送数据长度
* 输出参数:                                                              
*               unsigned long *pLen         发送数据长度
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               对使用RSP_UDP_GetDstConnectHandle方式建立的固定UDP连接，
*               数据发送时需要UDP接收端进行配合，否则可能提示数据发送失败。
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_Send( RSP_HANDLE local, RSP_HANDLE remote, const unsigned char *data, unsigned long *pLen );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_UDP_Server_Recv
* 功能描述:     UDP Server端接收数据 
* 输入参数:
*               RSP_HANDLE local            本地UDP资源标识
*               RSP_HANDLE remote           远端UDP资源标识
*               const unsigned char *data   接收数据缓存
*               unsigned long *pLen         接收数据长度
* 输出参数:                                                              
*               unsigned char *data         接收数据缓存
*               unsigned long *pLen         接收数据长度
* 返 回 值:                                                              
*               RSP_RT_SUCCESS  成功
*               其他            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_UDP_Recv( RSP_HANDLE local, RSP_HANDLE remote, unsigned char *data, unsigned long *pLen );

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: _RSP_UDP_IF_H */



