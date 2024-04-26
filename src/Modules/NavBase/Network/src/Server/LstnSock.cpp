// LstnSock.cpp : implementation file
//

//#include "stdafx.h"
#include "LstnSock.h"
#include "TcpSock.h"
#include "FrmChan.h"
#include "TcpSrvCom.h"

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif

/////////////////////////////////////////////////////////////////////////////
// Implementation of class "CListeningSocket".

CListeningSocket::CListeningSocket(CTcpServerCom* pNetCom): CCeSocket(/*FOR_LISTENING*/)
{
	m_pNetCom = pNetCom;
}

CListeningSocket::~CListeningSocket()
{
//	Detach();
}

//// Do not edit the following lines, which are needed by ClassWizard.
//#if 0
//BEGIN_MESSAGE_MAP(CListeningSocket, CSocket)
//	//{{AFX_MSG_MAP(CListeningSocket)
//	//}}AFX_MSG_MAP
//END_MESSAGE_MAP()
//#endif	// 0

/////////////////////////////////////////////////////////////////////////////
// CListeningSocket member functions

void CListeningSocket::OnAccept(int nErrorCode) 
{
	//TODO
//	CCeSocket::OnAccept(nErrorCode);
//
//	SOCKADDR_IN Peer;
//	int nPeerSize = sizeof(Peer);
//
//	// Make a new client socket for connection
//	CTcpSocket* pSocket = new CTcpSocket;
//
//	// Accept the connection
//   if (Accept(*pSocket, (SOCKADDR*)(&Peer), &nPeerSize))
//      if (m_pNetCom->AcceptConnect(pSocket, Peer))
//      {
//#if defined _WIN32_WCE
//         // To fix the bug of WinCE in OnReceive()
//         //   Check MSDN KB 253945 for detail
////         pSocket->m_bConnectCalled = TRUE;
//#endif
//         return;
//      }

   // error, delete the socket
//	delete pSocket;
}

void CListeningSocket::OnClose(int nErrorCode) 
{
      //TODO
     //CCeSocket::OnClose(nErrorCode);
    //	m_pChannel->DetachSocket();
}

BOOL CListeningSocket::Create(UINT uPort)
{
    if (!CSocket::Create(AF_INET, SOCK_STREAM))
    return FALSE;

    if(!Bind(uPort,AF_INET))
    return FALSE;

    if (!Listen())
    return FALSE;

    return TRUE;
}
