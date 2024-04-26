/*<FH+>************************************************************************/
/*                                                                            */
/* ��Ȩ����: Copyright (C) �������ɻ������Զ����ɷ����޹�˾                   */
/*                                                                            */
/* �ļ�����: public.h                                                         */
/* ����ժҪ: ��������ͷ�ļ�                                                   */
/* ����˵��:                                                                  */
/* ��ǰ�汾: 1.0                                                              */
/* ��    ��: Ǯ����                                                           */
/* �������: 2012-05-28                                                       */
/* �޸ļ�¼:                                                                  */
/*    �޸�����          �汾��        �޸���        �޸�����                  */
/* -------------------------------------------------------------------------- */
/*    2013-09-23        1.0           QianYizhou        �����ļ�                  */
/*<FH->************************************************************************/
#ifndef _RSP_DEFINE_H
#define _RSP_DEFINE_H

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

/******************************************************************************/
/*               #include������Ϊ��׼��ͷ�ļ����Ǳ�׼��ͷ�ļ���               */
/******************************************************************************/

/******************************************************************************/
/*                                 ��������                                   */
/******************************************************************************/

/******************************************************************************/
/*                                ȫ�ֺ궨��                                  */
/******************************************************************************/
/*
 * ����忨����
 */
//LX800 ���Ͱ忨
#define BD_TYPE_LX800           0
//E600  ���Ͱ忨
#define BD_TYPE_E600            1
//E3845 ���Ͱ忨
#define BD_TYPE_E3845           2

//��Ч���ļ�������
#define INVALID_FILE_DESCRIPTER (-1)                

/******************************************************************************/
/* ����ֵ�궨��                                                               */
/******************************************************************************/
#ifndef RSP_RETURN_VALUE
#define RSP_RETURN_VALUE

#define RSP_RT_ERROR_BASE   (-1000L)

#define RSP_RT_FAILED       (-1) 

#define RSP_RT_SUCCESS      (0) 

////////////////////////////////////////////////////////////////////////////////
//Common Error Info
#define RSP_RT_ERROR_INVALID_ARG    (RSP_RT_ERROR_BASE + 1) //������Ч

#define RSP_RT_ERROR_TIMEOUT        (RSP_RT_ERROR_BASE + 2) //��ʱ

#define RSP_RT_ERROR_MALLOC_FAILED  (RSP_RT_ERROR_BASE + 3) //mallocʧ��
////////////////////////////////////////////////////////////////////////////////
#define RSP_RT_ERROR_DEVICE_BASE                (RSP_RT_ERROR_BASE - 1000)    //�豸����base

#define RSP_RT_ERROR_DEVICE_ALREADY_OPENED      (RSP_RT_ERROR_DEVICE_BASE + 3) //�豸�Ѿ���

#define RSP_RT_ERROR_DEVICE_ALREADY_CLOSED      (RSP_RT_ERROR_DEVICE_BASE + 4) //�豸�Ѿ��ر�

#define RSP_RT_ERROR_DEVICE_OPEN_FAILED         (RSP_RT_ERROR_DEVICE_BASE + 5) //�豸��ʧ��

#define RSP_RT_ERROR_DEVICE_CLOSE_FAILED        (RSP_RT_ERROR_DEVICE_BASE + 6) //�豸�ر�ʧ��

#define RSP_RT_ERROR_DEVICE_WRITE_FAILED        (RSP_RT_ERROR_DEVICE_BASE + 7) //�豸дʧ��

#define RSP_RT_ERROR_DEVICE_READ_FAILED         (RSP_RT_ERROR_DEVICE_BASE + 8) //�豸��ʧ��

#define RSP_RT_ERROR_DEVICE_ASSIGN_RES_FAILED   (RSP_RT_ERROR_DEVICE_BASE + 9 )//�豸��Դ����ʧ��

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
/*                              ȫ��������������                              */
/******************************************************************************/
//typedef void *RSP_HANDLE;
#include <stdint.h>
typedef intptr_t    RSP_HANDLE;

//typedef unsigned int DWORD;
//typedef unsigned char BYTE;

/******************************************************************************/
/*                                ȫ�ֱ�������                                */
/******************************************************************************/

/******************************************************************************/
/*                                �ⲿ���ö���                                */
/******************************************************************************/

/******************************************************************************/
/*                                ȫ�ֺ���ԭ��                                */
/******************************************************************************/

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif /* end of include guard: _RSP_DEFINE_H */
