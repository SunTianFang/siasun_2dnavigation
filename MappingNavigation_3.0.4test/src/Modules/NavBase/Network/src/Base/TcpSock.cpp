// DataSock.cpp : implementation file
//

#include "stdafx.h"
#include "TcpSock.h"
#include "TcpChannel.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTcpSocket".

CTcpSocket::CTcpSocket(CTcpChannel* pChannel)
{
	m_pChannel = pChannel;
}

/////////////////////////////////////////////////////////////////////////////
// CTcpSocket member functions                                         

void CTcpSocket::SetChannel(CTcpChannel* pChannel)
{
   m_pChannel = pChannel;
}

void CTcpSocket::OnReceive(int nErrorCode) 
{
#ifdef WINDOWS_PLATFORM_USING

	CCeSocket::OnReceive(nErrorCode);
	if (m_pChannel != NULL)
	{
		m_pChannel->DoReceive();
	}
#elif defined(LINUX_PLATFORM_USING)
	if (m_pChannel != NULL)
	{
		m_pChannel->DoReceive();
	}
#endif
}

void CTcpSocket::OnClose(int nErrorCode) 
{

#ifdef WINDOWS_PLATFORM_USING
	CCeSocket::OnClose(nErrorCode);
#if 0
	if (m_pChannel != NULL)
	{
		m_pChannel->DetachSocket();
	}
#endif
#elif defined(LINUX_PLATFORM_USING)

#endif


}
