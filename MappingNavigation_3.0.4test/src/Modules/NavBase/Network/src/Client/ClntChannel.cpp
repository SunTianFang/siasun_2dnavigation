//                                - CLNTCHANNEL.CPP -
//
//   Implementatin of class "CClientChannel".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#include <stdafx.h>
#include "ClntChannel.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CClientChannel".

CClientChannel::CClientChannel(BOOL bFramed, BOOL bAscii) :
CTcpChannel(bFramed, bAscii)
{
	m_uLocalPort = 0;
	m_nRemotePort = -1;
}

CClientChannel::~CClientChannel()
{
	if (m_pSocket != NULL && m_bConnected)
	{

#ifdef WINDOWS_PLATFORM_USING
//WinSocket
		m_pSocket->Disconnect();
#elif defined(LINUX_PLATFORM_USING)
//CSocket
		m_pSocket->Close();
#endif	
	}
}

//
//   Set local ports.
//
void CClientChannel::SetLocalPort(UINT uPort)
{
   m_uLocalPort = uPort;
}

//
//   Set Host computer IP address and port.
//
void CClientChannel::SetHostPort(int nPort)
{
	m_nRemotePort = nPort;
}

//
//   Set the host computer IP address.
//
void CClientChannel::SetHostIpAddr(const TCHAR* szAddr)
{
	_tcsncpy(m_szIpAddr, szAddr, 16);
}

//
//   Create (or re-create) the object.
//
BOOL CClientChannel::CreateSocket()
{
   if (m_pSocket != NULL)
      DetachSocket();
   
	if (!CTcpChannel::CreateSocket())
		return FALSE;

	if (m_pSocket != NULL)
	{
#ifdef WINDOWS_PLATFORM_USING
//WinSocket
		if(m_pSocket->Create(SOCK_STREAM, 4096))
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
#elif defined(LINUX_PLATFORM_USING)
//CSocket
        return m_pSocket->Create();
#endif

	}
	else
	{
		return FALSE;
	}
}

//
//   Connect the client to a server.
//
BOOL CClientChannel::Connect()
{
	BOOL bError = FALSE;

	// If the socket does not exist, create it
	if (m_pSocket == NULL)
	{
		if (m_nRemotePort < 0 || !CreateSocket())
			return FALSE;
	}

	// Add the time out for the socket
	if(m_pSocket!=NULL)
	{
		if(!m_pSocket->SetTimeOut(10000))
		{
			// Error Handling...for some reason, we could not setup the timer.
		}
	}
	else
	{
		return FALSE;
	}


#ifdef WINDOWS_PLATFORM_USING
	if(m_pSocket!=NULL)
	{
		//CacyncSocket => CESocket
#if 0
		//CSocket
		if (!m_pSocket->Connect(m_szIpAddr, m_nRemotePort))
#else
		//WinSocket
		CString cstrIpAddr(m_szIpAddr);
		if(!m_pSocket->Connect(cstrIpAddr, m_nRemotePort, 10))  //sencods
#endif
		{
			bError = TRUE;

			int nError = GetLastError();
			if(nError==WSAEINTR)
			{
			}
			else
			{
				// Do other error processing.
			}

			m_bConnected = FALSE;
			//return FALSE;
		}
	}
	else
	{
		return FALSE;

	}
#elif defined(LINUX_PLATFORM_USING)
        /*******************************/ //bug = new no delete
//	size_t len= strlen(m_szIpAddr) + 1;
//	char *m_szIpAddrTemp = new char[len];

//	strncpy(m_szIpAddrTemp,m_szIpAddr,len);
	/*****************************************/
	if(m_pSocket!=NULL)
	{
        if (!m_pSocket->Connect(/*m_szIpAddrTemp*/m_szIpAddr, m_nRemotePort))
		{
			//			bError = TRUE;
			//
			//			int nError = GetLastError();
			//			if(nError==WSAEINTR)
			//			{

			//			}
			//			else
			//			{
			//				// Do other error processing.
			//			}

			m_bConnected = FALSE;
			return FALSE;
		}
	}
	else
	{
		return FALSE;

	}
#endif
	if(m_pSocket!=NULL)
	{
		if(!m_pSocket->KillTimeOut())
		{
			// Error Handling...for some reason the timer could not be destroyed 
		}
	}
	else
	{
		return FALSE;
	}

	if (bError)
		return FALSE;

	m_bConnected = TRUE;


#ifdef WINDOWS_PLATFORM_USING
#elif defined(LINUX_PLATFORM_USING)
	//***add  one thread for listen soceket fd***//
	m_pSocket->Start();
	/*****************************************************/
#endif



	return TRUE;
}

//
//   Detach the socket from the channel.
//
void CClientChannel::DetachSocket()
{
   CTcpChannel::DetachSocket();

	m_pSend = m_SBuf;
	m_pRecv = m_RBuf;
	m_nRecvCount = 0;

	m_bConnected = FALSE;
}

void CClientChannel::Disconnect()
{
	CTcpChannel::Disconnect();
}
