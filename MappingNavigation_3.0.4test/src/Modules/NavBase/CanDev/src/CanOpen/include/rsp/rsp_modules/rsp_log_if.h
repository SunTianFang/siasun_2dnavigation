#ifndef _RSP_LOG_SEND_IF_H
#define _RSP_LOG_SEND_IF_H

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

//#include <stdarg.h>
#include <unistd.h>
#include <sys/syscall.h>

#include "rsp_define.h"

#define RSP_LOG_EMERG           0
#define RSP_LOG_ALERT           1
#define RSP_LOG_CRIT            2
#define RSP_LOG_ERR             3
#define RSP_LOG_WARNING         4
#define RSP_LOG_NOTICE          5
#define RSP_LOG_INFO            6
#define RSP_LOG_DEBUG           7

#define RSP_LOG_OPT_IDENT       0x01
#define RSP_LOG_OPT_PID         0x02
#define RSP_LOG_OPT_TID         0x04
#define RSP_LOG_OPT_CONS        0x08
/*<FUNC+>**********************************************************************
* 函数名称: RSP_LOG_Open
* 功能描述: 打开RSP_LOG发送端
* 输入参数: 
*               const char *ident       用户发送进程标识
*               int logOpt              用户发送进程选项
* 输出参数: 
*               void
* 返 回 值:
*               RSP_RT_SUCCESS:     成功
*               其他：              错误码
*               
* 操作流程:
* 其它说明:
* 修改记录:
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-10-21           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_LOG_Open( const char *ident, int logOpt );

/*<FUNC+>**********************************************************************
* 函数名称: RSP_LOG_SetLevel                                                             
* 功能描述: 设置用户进行的输出信息等级
* 输入参数: 
*               int level               用户设置的输出信息等级
* 输出参数: 
*               void
* 返 回 值:     
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-10-21           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
void RSP_LOG_SetLevel( int level );

/*<FUNC+>**********************************************************************
* 函数名称: RSP_LOG_GetLevel                                                             
* 功能描述: 获取用户当前的输出信息等级
* 输入参数: 
*               void
* 输出参数: 
*               int *pLevel         当前的用户信息指针
* 返 回 值:     
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-10-21           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_LOG_GetLevel( void );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_LOG_SendMsg_Base                                                             
* 功能描述:     RSP_LOG基本信息输出
* 输入参数: 
*               level               信息输出等级
*               char *format        信息输出格式
*               ...                 可变参数
* 输出参数: 
*               void
* 返 回 值:     
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
*               按照普通的printf类函数调用即可
*               Example:
*                   char *var1 = ...;
*                   int var2 = ...;
*                   RSP_LOG_SendMsg_Base( "[%s]user debug info is %d.", var1, var2 );
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-10-21           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
//int RSP_LOG_SendMsg_Base( char *format, ... );
//int RSP_LOG_SendMsg_Base( int level, char *format, ... );
int RSP_LOG_SendMsg_Base( int level, void *buf, int len );

#define RSP_LOG_MAX_IDENT       128
#define RSP_LOG_MAX_TEXT        256
typedef struct t_rsp_log_msg {
    long int rsp_msg_type;
    char text[RSP_LOG_MAX_TEXT];
}T_RSP_LOG_MSG,
*PTRSP_LOG_MSG;
/*      用户设置的进程Ident     */
extern char ac_gpi_LogIdent[];
/*      用户设置的日志输出等级  */
extern int gpi_LogLevel;
/*      输出信息选项            */
extern int gpi_IPCOpt;

#if 0/*{{{*/
#define RSP_LOG_SendMsg_PID( level, format, ... ) \
            { \
                if( gpi_IPCOpt & RSP_LOG_OPT_PID ) { \
                    RSP_LOG_SendMsg_Base( level, "[PID:%d]"format, getpid(), ##__VA_ARGS__ ); \
                } else { \
                    RSP_LOG_SendMsg_Base( level, format, ##__VA_ARGS__ ); \
                } \
            }

#define RSP_LOG_SendMsg_TID( level, format, ... ) \
            { \
                if( gpi_IPCOpt & RSP_LOG_OPT_TID ) { \
                    RSP_LOG_SendMsg_PID( level, "[TID:%d]"format, syscall(SYS_gettid), ##__VA_ARGS__ ); \
                } else { \
                    RSP_LOG_SendMsg_PID( level, format, ##__VA_ARGS__ ); \
                } \
            }

#define RSP_LOG_SendMsg_IDENT( level, format, ... ) \
            { \
                if( gpi_IPCOpt & RSP_LOG_OPT_IDENT ) { \
                    RSP_LOG_SendMsg_TID( level, "[%s]:"format, ac_gpi_LogIdent, ##__VA_ARGS__ ); \
                } else { \
                    RSP_LOG_SendMsg_TID( level, format, ##__VA_ARGS__ ); \
                } \
            }
#endif/*}}}*/

#define RSP_LOG_SendMsg_PID( level, format, ... )       { \
    T_RSP_LOG_MSG msg; \
    snprintf( msg.text, RSP_LOG_MAX_TEXT, "[PID:%d]"format, getpid(), ##__VA_ARGS__ ); \
    RSP_LOG_SendMsg_Base( level, (void *)&msg, RSP_LOG_MAX_TEXT ); \
}

#define RSP_LOG_SendMsg_TID( level, format, ... )       { \
    RSP_LOG_SendMsg_PID( level, "[TID:%lu]"format, syscall(SYS_gettid), ##__VA_ARGS__ ); }

#define RSP_LOG_SendMsg_IDENT( level, format, ... )     RSP_LOG_SendMsg_TID( level, "[%s]:"format, ac_gpi_LogIdent, ##__VA_ARGS__ )

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_LOG
* 功能描述:     RSP_LOG信息输出
* 输入参数: 
*               int level           信息等级
*               char *format        信息输出格式
*               ...                 可变参数
* 输出参数:                                                              
*               void
* 返 回 值:                                                              
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
*               按照普通的printf类函数调用即可
*               Example:
*                   char *var1 = ...;
*                   int var2 = ...;
*                   RSP_LOG( "[%s]user debug info is %d.", var1, var2 );
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2013-10-21           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
#define RSP_LOG( level, format, ... )       RSP_LOG_SendMsg_TID( level, format"\n", ##__VA_ARGS__ )

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_LOG_Service_Start
* 功能描述:     启动日志接收服务
* 输入参数:     
*               const char *logAim  日志信息缓存目标
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               对参数logAim
*                   如果logAim格式为 IP（XX.XX.XX.XX）格式，将采用UDP方式将日志发送到远端
*                   如果logAim采用其他格式，将认为logAim为日志文件的缓存路径，并采用日志缓存方式记录
*               该接口调用后，将阻塞式接收数据
*               可以采用#killall -SIGUSR2 PROCESSNAME 方式终止该接口执行
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*<FUNC->**********************************************************************/
int RSP_LOG_Service_Start( const char *logAim );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_LOG_Service_Stop
* 功能描述:     终止日志接收服务
* 输入参数:                                                              
*               NULL
* 输出参数:                                                              
*               NULL
* 返 回 值:                                                              
*               RSP_RT_SUCCESS      成功
*               其他：              错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*<FUNC->**********************************************************************/
int RSP_LOG_Service_Stop( void );

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: _RSP_LOG_SEND_IF_H */

