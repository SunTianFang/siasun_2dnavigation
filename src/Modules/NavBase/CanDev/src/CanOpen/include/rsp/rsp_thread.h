#ifndef __GPI_THREAD_H__
#define __GPI_THREAD_H__

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

#include "rsp.h"

//typedef void *RSP_HANDLE;

int RSP_Thread_Init( 
        void *(pEntry)(void *),
        void *arg,
        unsigned int priority, 
        RSP_HANDLE *pHandle );

int RSP_Thread_Wait( RSP_HANDLE handle, void **pRtValue );

int RSP_Thread_Stop( RSP_HANDLE handle );

//int RSP_Thread_Uninit( RSP_HANDLE *pHandle, void **pRtValue );
int RSP_Thread_Uninit( RSP_HANDLE *pHandle );

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: __GPI_THREAD_H__ */



