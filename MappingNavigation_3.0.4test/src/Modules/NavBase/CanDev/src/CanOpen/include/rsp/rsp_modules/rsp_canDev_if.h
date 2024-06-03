/*<FH+>************************************************************************/
/*                                                                            */
/* ��Ȩ����: Copyright (C) ���������Զ����ɷ����޹�˾                         */
/*                                                                            */
/*                                                                            */
/*  �ļ�����:rsp_can_if.h                                                              */
/*  ����ժҪ:                                                                 */
/*  ����˵��:                                                                 */
/*  ��ǰ�汾:                                                                 */
/*  ��    ��:QianYizhou                                                       */
/*  �������:2013��09��17��                                                   */
/*  �޸ļ�¼:                                                                 */
/*    �޸�����          �汾��        �޸���        �޸�����                  */
/* -------------------------------------------------------------------------- */
/*    2013��09��18��          V1.0        QianYizhou        �����ļ�          */
/*<FH->************************************************************************/

#ifndef _RSP_CAN_IF_H
#define _RSP_CAN_IF_H

#include "rsp_define.h" 

#ifdef  __cplusplus
extern "C"
{
#endif//__cplusplus

//��Դ�����ʶ
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
* ��������:     RSP_CAN_Open
* ��������:     ��CAN�豸                                                             
* �������: 
*               unsigned char ucChNum   CAN�豸�˿ڣ���1��ʼ��
* �������: 
*               RSP_HANDLE *pHandle     ��Դ���
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Open( unsigned char ucChNum, RSP_HANDLE *pHandle );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_Close
* ��������:     �ر�CAN�豸
* �������: 
*               RSP_HANDLE handle       �豸��Դ���
* �������:     
*               NULL
* �� �� ֵ:     
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Close( RSP_HANDLE handle );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_Config
* ��������:     ����CAN�豸                                                             
* �������: 
*               RSP_HANDLE handle       ��Դ���
*               int nBaundRate          ������
*                   1000    -- 1M  kps 
*                   500     -- 500 Kps
*                   250     -- 250 kps
*                   100     -- 100 kps
*                   20      -- 20  kps
*               int nMsgType            ��������
* �������: 
*               NULL
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_Config( RSP_HANDLE handle, int nBaundRate, int nMsgType  );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_SendBlock
* ��������:     CAN��������(����ʽ)
* �������: 
*               RSP_HANDLE handle   CAN��Դ���
*               TPCANMsg *ptSendMsg CAN��������
* �������:     
*               NULL
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_SendBlock( RSP_HANDLE handle, TPCANMsg* ptSendMsg );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_SendTimeOut
* ��������:     CAN��������(������ʽ)
* �������: 
*               RSP_HANDLE handle   CAN��Դ���
*               const TPCANMsg *ptSendMsg CAN��������
*               int nMicroSeconds   ��ʱʱ�䣨��λ:ms��
* �������:     
*               NULL
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_SendTimeOut( RSP_HANDLE hHandle, TPCANMsg *ptSendMsg, int nMicroSeconds );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_RecvBlock
* ��������:     CAN�������ݣ�����ʽ��
* �������: 
*               RSP_HANDLE handle       CAN��Դ���
* �������:     
*               TPCANRdMsg *ptRecvMsg   CAN�������ݽṹ��
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_RecvBlock( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg );

/*<FUNC+>**********************************************************************
* ��������:     RSP_CAN_RecvTimeOut
* ��������:     CAN�������ݣ�������ʽ��
* �������: 
*               RSP_HANDLE handle       CAN��Դ���
*               int nMicroSeconds       ��ʱʱ��
* �������:     
*               TPCANRdMsg *ptRecvMsg   CAN�������ݽṹ��
* �� �� ֵ: 
*               RSP_RT_SUCCESS    �ɹ�
*               ����              ������
* ��������:                                                              
* ����˵��:                                                              
* �޸ļ�¼:                                                              
*    �޸�����          �汾��        �޸���        �޸�����                  
* -------------------------------------------------------------------------- 
*   2013-9-18           V1.0        QianYizhou      create
*<FUNC->**********************************************************************/
int RSP_CAN_RecvTimeOut( RSP_HANDLE handle, TPCANRdMsg *ptRecvMsg, int nMicroSeconds );

#ifdef  __cplusplus
}
#endif//__cplusplus

#endif//_RSP_CAN_IF_H



