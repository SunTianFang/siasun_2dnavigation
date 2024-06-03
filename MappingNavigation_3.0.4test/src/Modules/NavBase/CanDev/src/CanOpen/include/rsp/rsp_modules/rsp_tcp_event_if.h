/*<FH+>************************************************************************/
/*                                                                            */
/* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         */
/*                                                                            */
/*                                                                            */
/*  文件名称:rsp_tcp_moudle.h                                                 */
/*  内容摘要:                                                                 */
/*  其它说明:                                                                 */
/*  当前版本:                                                                 */
/*  作    者:LiuQingjie                                                       */
/*  完成日期:2016年10月17日                                                   */
/*  修改记录:                                                                 */
/*      修改日期            版本号        修改人            修改内容          */
/* -------------------------------------------------------------------------- */
/*      2016年10月17日      V1.0          LiuQingjie        创建文件          */
/*<FH->************************************************************************/
#ifndef _RSP_TCP_MODULE_H_
#define _RSP_TCP_MODULE_H_

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

#include <stdbool.h>

//typedef intptr_t    RSP_HANDLE;
#include "rsp_define.h"

typedef void (* client_cb_connect)( RSP_HANDLE handle );
typedef void (* client_cb_fail_to_connect )( RSP_HANDLE handle );
typedef void (* client_cb_connection_closed )( RSP_HANDLE handle );
typedef void (* client_cb_receive_from_server)( RSP_HANDLE handle );
typedef void (* server_cb_listen)( RSP_HANDLE handle );
typedef void (* server_cb_disconnect )( RSP_HANDLE handle );
typedef void (* server_cb_recv)( RSP_HANDLE handle );

typedef struct t_client_opt
{
    const char*                   ip; // server ip
    int                           port; // server port
    bool                          ifReconnect;
    client_cb_connect             connector; // called when client connected   
    client_cb_receive_from_server receiver; // called when received data from server
    client_cb_connection_closed   connection_closed;
    client_cb_fail_to_connect     fail_to_connect;
} T_CLIENT_OPT,
*PT_CLIENT_OPT;

typedef struct t_server_opt
{
    char*                         ip; // server ip
    int                           port; // server port
    int                           nMaxClientCount; // max connection
    server_cb_listen              listener; // called when client connected
    server_cb_disconnect          disconnecter; // called when client disconnected
    server_cb_recv                receiver; // called when received data from client
} T_SERVER_OPT,
*PT_SERVER_OPT;

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_SendData
* 功能描述:     TCP发送数据
* 输入参数:
*               RSP_HANDLE handle               客户端资源标识
*               char* data                      发送数据缓存
*               int size                        发送数据长度
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               0                               成功
*               其他                            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
int RSP_TCP_EVENT_SendData( RSP_HANDLE handle, char* data, int size );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_RecvData
* 功能描述:     TCP接收数据
* 输入参数:
*               RSP_HANDLE handle               客户端资源标识
*               char* data                      接收数据缓存
*               int size                        最多接收数据长度
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               int                             实际接收的字节数
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
int RSP_TCP_EVENT_RecvData( RSP_HANDLE handle, char* data, int size );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_InitClient
* 功能描述:     TCP Client端初始化
* 输入参数:
*               PT_CLIENT_OPT pOpt              客户端结构体
* 输出参数:                                                              
*               RSP_HANDLE *pClient             客户端资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS                  成功
*               其他                            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
int RSP_TCP_EVENT_InitClient( PT_CLIENT_OPT pOpt, RSP_HANDLE *pClient );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_ConnectAndLoop
* 功能描述:     TCP Client端连接服务器
* 输入参数:
*               RSP_HANDLE handle             客户端资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_ConnectAndLoop( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_UninitClient
* 功能描述:     TCP Client端反初始化
* 输入参数:
*               RSP_HANDLE handle             客户端资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_UninitClient( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_QuitClientLoop
* 功能描述:     TCP Client端退出循环
* 输入参数:
*               RSP_HANDLE handle             客户端资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_QuitClientLoop( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_IsConnected
* 功能描述:     TCP Client端是否已连接
* 输入参数:
*               RSP_HANDLE handle             客户端资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
bool RSP_TCP_EVENT_IsConnected( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_GetClientConn
* 功能描述:     TCP Client端取得链接资源
* 输入参数:
*               RSP_HANDLE handle             客户端资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
RSP_HANDLE RSP_TCP_EVENT_GetClientConn( RSP_HANDLE client );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_GetServerInfo
* 功能描述:     TCP Client端取得所连接的服务器的信息
* 输入参数:
*               RSP_HANDLE client_handle     客户端资源标识
* 输出参数:                                                              
*               char* ip                     服务器ip
*               int* port                    服务器端口号
* 返 回 值:
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* ----------------------------------------------------------------------------- 
*   2016-10-17         V1.0          LiuQingjie     create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_GetServerInfo( RSP_HANDLE handle, char *ip, int* port );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_ServerInit
* 功能描述:     TCP Server端初始化
* 输入参数:
*               PT_SERVER_OPT pOpt              服务器结构体
* 输出参数:                                                              
*               RSP_HANDLE *pServer             服务器资源标识
* 返 回 值:                                                              
*               RSP_RT_SUCCESS                  成功
*               其他                            错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
int RSP_TCP_EVENT_InitServer( PT_SERVER_OPT pOpt, RSP_HANDLE *pServer );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_ServerUninit
* 功能描述:     TCP Server端反初始化
* 输入参数:
*               RSP_HANDLE handle             服务器资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_UninitServer( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_ServerLoop
* 功能描述:     TCP Server端循环监听可读事件
* 输入参数:
*               RSP_HANDLE handle             服务器资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_ServerLoop( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_ServerLoop
* 功能描述:     TCP Server端退出循环
* 输入参数:
*               RSP_HANDLE handle             服务器资源标识
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2016-10-17           V1.0        LiuQingjie      create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_QuitServerLoop( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_EVENT_Broadcast
* 功能描述:     TCP Server端向所有客户端发送广播
* 输入参数:
*               RSP_HANDLE server_handle     服务器资源标识
*               char* data                   数据
*               int size                     数据大小
* 输出参数:                                                              
*               NULL
* 返 回 值:
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* ----------------------------------------------------------------------------- 
*   2016-10-17         V1.0          LiuQingjie     create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_Broadcast( RSP_HANDLE server_handle, char* data, int size );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_TCP_GetClientListInfo
* 功能描述:     TCP Server端取得所连接的所有客户端的信息
* 输入参数:
*               RSP_HANDLE server_handle     服务器资源标识
* 输出参数:
*               int* nCurrentClientCount     当前连接的客户端数                                                  
*               char*[] ip_list              客户端ip列表
*               int* server_port             服务器端口号
* 返 回 值:
*               NULL
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* ----------------------------------------------------------------------------- 
*   2016-10-17         V1.0          LiuQingjie     create
*<FUNC->**********************************************************************/
void RSP_TCP_EVENT_GetClientListInfo( RSP_HANDLE handle, int* nCurrentClientCount, char* ip_list[], int* server_port );

#ifdef  __cplusplus
}
#endif//__cplusplus


#endif // #ifndef _RSP_TCP_MODULE_H_
