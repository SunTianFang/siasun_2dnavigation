//                                - TCPCHANNEL.CPP -
//
//   Implementatin of class "CTcpChannel".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#include <stdafx.h>
#include "TcpChannel.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTcpChannel".

CTcpChannel::CTcpChannel(BOOL bFramed, BOOL bAscii) : CFrameChannel(bFramed, bAscii)
{
	m_pSocket = NULL;
	m_bConnected = FALSE;
}

CTcpChannel::~CTcpChannel()
{
   DetachSocket();
}

BOOL CTcpChannel::CreateSocket()
{
	m_TcpCritSection.Lock();
	BOOL bReturn = FALSE;
	m_pSocket = new CTcpSocket(this);
	bReturn =  (m_pSocket != NULL);
	m_TcpCritSection.Unlock();
	return bReturn;
}

//
//   Detach: Detaches the client with its current socket.
//
void CTcpChannel::DetachSocket()
{
	m_TcpCritSection.Lock();
	if (m_pSocket != NULL)
   {
		#ifdef WINDOWS_PLATFORM_USING
				//WinSocket
			   m_pSocket->Disconnect();
			   delete m_pSocket;
		#elif defined(LINUX_PLATFORM_USING)
			   
//CSocket
               if(m_pSocket)
               {
                    m_pSocket->Detach();
                    delete m_pSocket;
                    m_pSocket = NULL;
               }
#endif
   }

	m_pSocket = NULL;
	m_bConnected = FALSE;
	m_TcpCritSection.Unlock();
}

int CTcpChannel::Send(UCHAR* pBuf, int nBufLen)
{
	m_TcpCritSection.Lock();
	int nReturn = SOCKET_ERROR;
	if (m_pSocket != NULL && m_bConnected)
	{
#ifdef WINDOWS_PLATFORM_USING
		//WinSocket
		nReturn = m_pSocket->Send((char*)pBuf, nBufLen);

#elif defined(LINUX_PLATFORM_USING)
//CSocket
		nReturn = m_pSocket->Send(pBuf, nBufLen);
#endif	
	}
	else
		nReturn = SOCKET_ERROR;
	m_TcpCritSection.Unlock();
	return nReturn;
}

// dq VISION
int CTcpChannel::Send_Cam(UCHAR* pBuf, int nBufLen)
{
    m_TcpCritSection.Lock();
    int nReturn = SOCKET_ERROR;
    if (m_pSocket != NULL && m_bConnected)
    {
#ifdef WINDOWS_PLATFORM_USING
        //WinSocket
        nReturn = m_pSocket->Send((char*)pBuf, nBufLen);

#elif defined(LINUX_PLATFORM_USING)
//CSocket
        nReturn = m_pSocket->Send(pBuf, nBufLen);
#endif
    }
    else
        nReturn = SOCKET_ERROR;
    m_TcpCritSection.Unlock();
    return nReturn;
}

int CTcpChannel::Receive(UCHAR* pBuf, int nBufLen)
{
	m_TcpCritSection.Lock();
	int nReturn = SOCKET_ERROR;
	if (m_pSocket != NULL && m_bConnected)
	{

#ifdef WINDOWS_PLATFORM_USING
//WinSocket
		nReturn = m_pSocket->Read((char*)pBuf, nBufLen);
#elif defined(LINUX_PLATFORM_USING)//CSocket
		nReturn = m_pSocket->Receive(pBuf, nBufLen);

#endif
	}
	else
		nReturn = SOCKET_ERROR;
	m_TcpCritSection.Unlock();
	return nReturn;
}

void CTcpChannel::Disconnect()
{
#ifdef WINDOWS_PLATFORM_USING
	m_TcpCritSection.Lock();
	if (m_pSocket != NULL)
	{
		m_pSocket->Disconnect();
	}
	m_TcpCritSection.Unlock();
#elif defined(LINUX_PLATFORM_USING)

#endif

}
