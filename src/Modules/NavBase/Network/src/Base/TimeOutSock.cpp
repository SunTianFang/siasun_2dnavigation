// TimeOutSock.cpp : implementation file
//
//   Author: Lv Xiangren
//   Date:   2013. 06. 22
//

#include "stdafx.h"
#include "TimeOutSock.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTimeOutSock".

CTimeOutSocket::CTimeOutSocket()
{
	m_dwTimeStart = 0;
	m_uTimeOut = 0;
	m_bError = FALSE;
}

//BOOL CTimeOutSocket::OnMessagePending()
//{
//	if(m_dwTimeStart)
//	{
//		if (GetTickCount() - m_dwTimeStart > m_uTimeOut)
//		{
//			m_bError = TRUE;
//			CancelBlockingCall();
//
//
//			return FALSE;  // No need for idle time processing.
//		}
//	}
//
//	return CSocket::OnMessagePending();
//}

BOOL CTimeOutSocket::SetTimeOut(DWORD uTimeOut)
{
	m_dwTimeStart = GetTickCount() - 100;
	m_uTimeOut = uTimeOut;
	m_bError = FALSE;

	return TRUE;
}

BOOL CTimeOutSocket::KillTimeOut()
{
	m_dwTimeStart = 0;
	m_uTimeOut = 0;
	m_bError = FALSE;

	return TRUE;
}


