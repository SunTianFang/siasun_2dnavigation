/*<FH+>***********************************************************************
*                                                                            
* 版权所有: Copyright (C) 沈阳新松自动化股份有限公司                         
*                                                                            
*                                                                            
*  文件名称: rsp_am_if.h                                                   
*  内容摘要: RSP(Robot Software Platform) Alarm Module 头文件
*  其它说明:                                                             
*  当前版本:                                                             
*  作    者:                                                               
*  完成日期:                                                               
*  修改记录:                                                                 
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-19           V1.0        QianYizhou      Create
*<FH->************************************************************************/
#ifndef __RSP_AM_IF_H
#define __RSP_AM_IF_H

#include "rsp.h"

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Init
* 功能描述:     RSP Alarm Module 初始化
* 输入参数: 
*               unsigned long bufSize   post code的缓存大小
* 输出参数:
*               RSP_HANDLE *pHandle     AM模块句柄
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               入参bufSize不是code－info的大小限制，而是postcode的缓存大小
*               该模块对code－info的大小无限制
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Init( unsigned long bufSize, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Uninit
* 功能描述:     RSP Alarm Module 反初始化
* 输入参数: 
*               RSP_HANDLE *pHandle     AM模块句柄
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Uninit( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Register
* 功能描述:     向am模块注册code－info 
* 输入参数: 
*               RSP_HANDLE  handle      AM模块句柄
*               int code                键值对中的Key
*               const char *info        键值对中的value
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               入参中的info不能为局部变量
*               对同样的key，注册不同的value，将更新该key的value
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Register( RSP_HANDLE handle, int code, const char *info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_UnRegister
* 功能描述:     注销key-value对
* 输入参数: 
*               RSP_HANDLE  handle      AM模块句柄
*               int code                键值对中的Key
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               当module中code不存在，也将返回RSP_RT_SUCCESS
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_UnRegister( RSP_HANDLE handle, int code );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Update
* 功能描述:     向am模块更新code－info 
* 输入参数: 
*               RSP_HANDLE  handle      AM模块句柄
*               int code                键值对中的Key
*               const char *info        键值对中的value
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               入参中的info不能为局部变量
*               若key不存在，将向module插入key-value
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Update( RSP_HANDLE handle, int code, const char *info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Find
* 功能描述:     查找am中key对应的value
* 输入参数: 
*               RSP_HANDLE  handle      AM模块句柄
*               int code                键值对中的Key
* 输出参数:
*               const char **info       指向键值对中的value指针
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Find( RSP_HANDLE handle, int code, const char **info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Check
* 功能描述:     检测am中指定key是否存在
* 输入参数: 
*               RSP_HANDLE  handle      AM模块句柄
*               int code                键值对中的Key
* 输出参数:
*               int *pResult            指示code是否存在
*                                       1: 存在， 0： 不存在
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Check( RSP_HANDLE handle, int code, int *pResult );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_Size
* 功能描述:     获取am模块当前key-value的规模 
* 输入参数: 
*               RSP_HANDLE handle       AM模块句柄
* 输出参数:
*               int *pSize              am模块当前key-value的规模
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_Size( RSP_HANDLE handle, int *pSize );

//int RSP_AM_Foreach( RSP_HANDLE handle, RSP_AM_Func func, RSP_AM_POINTER *data );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_PostCode
* 功能描述:     向am模块投递code 
* 输入参数: 
*               RSP_HANDLE handle       AM模块句柄
*               int code                key
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               缓存post code的大小，受到RSP_AM_Init入参bufSize的限制
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_PostCode( RSP_HANDLE handle, int code );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_GetCurrentInfo
* 功能描述:     获取当前投递的code所对应的info
* 输入参数: 
*               RSP_HANDLE handle       AM模块句柄
* 输出参数:
*               const char **info       指向键值对中的value指针
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               需要向上获取投递的info时，需要首先调用RSP_AM_GetCurrentInfo一次，
*               需要向上获取投递的info时，需要首先调用RSP_AM_GetCurrentInfo一次，
*               当有postcode更新时，该接口自动最新加入code随对应的info
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_GetCurrentInfo( RSP_HANDLE handle, const char **info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_GetPrevInfo
* 功能描述:     获取上一个投递的code所对应的info
* 输入参数: 
*               RSP_HANDLE handle       AM模块句柄
* 输出参数:
*               const char **info       指向键值对中的value指针
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               需要向上获取投递的info时，需要首先调用RSP_AM_GetCurrentInfo一次，
*               再调用RSP_AM_GetPrevInfo
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_GetPrevInfo( RSP_HANDLE handle, const char **info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_GetNextInfo
* 功能描述:     获取下一个投递的code所对应的info
* 输入参数: 
*               RSP_HANDLE handle       AM模块句柄
* 输出参数:
*               const char **info       指向键值对中的value指针
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
*               需要向下获取投递的info时，需要首先调用RSP_AM_GetCurrentInfo一次，
*               再调用RSP_AM_GetNextInfo
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_GetNextInfo( RSP_HANDLE handle, const char **info );

/*<FUNC+>**********************************************************************
* 函数名称:     RSP_AM_ClearAllInfo
* 功能描述:     清除当前buf中所有的postcode
* 输入参数: 
*               RSP_HANDLE handle     AM模块句柄
* 输出参数:
*               NULL
* 返 回 值:     
*               RSP_RT_SUCCESS:         成功
*               其他：                  错误码
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*   2014-3-17           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_AM_ClearAllInfo( RSP_HANDLE handle );


#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: __RSP_AM_IF_H */

