//                      - SRVCHANNEL.CPP -
//
//   Implementatin of class "CTcpServerChannel".
//
//   Author: Zhang Lei
//   Date:   2006. 11. 10
//

#include "stdafx.h"
#include "TcpSrvChannel.h"
#include "TcpSrvCom.h"
#include"Tools.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTcpServerChannel".

CTcpServerChannel::CTcpServerChannel(CTcpServerCom* pServerCom, int nId, TCHAR* szIpAddr) :
CTcpChannel(TRUE,FALSE)
{
	// The IP address string must be shorter than 16
	ASSERT(_tcslen(szIpAddr) < 16);
   ASSERT(pServerCom != NULL);

   m_pServerCom = pServerCom;
	m_nId = nId;
	_tcscpy(m_szIpAddr, szIpAddr);
	m_pSocket = NULL;
	m_pSend = m_SBuf;
	m_pRecv = m_RBuf;
	m_nRecvCount = 0;
}

//
//   SetSocket: Attaches the client to the specified socket.
//
void CTcpServerChannel::AttachSocket(CTcpSocket* pSocket)
{
	ASSERT(pSocket != NULL);
	if (pSocket == NULL)
		return;

	m_pSocket = pSocket;
	m_pSocket->SetChannel(this);
	m_bConnected = TRUE;
}

//
//   Detach: Detaches the client with its current socket.
//
void CTcpServerChannel::DetachSocket()
{
   // Close the channel
   CTcpChannel::DetachSocket();

	// Remove the channel from the active channel list
   VERIFY(m_pServerCom->CloseChannel(m_nId));
}
