//                       - SERVERCOM.CPP -
//
//   Implementatin of class "CTcpServerCom", server version.
//
//   Author: Zhang Lei
//   Date:   2006. 12. 28
//

//#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include<stdlib.h>
#include <arpa/inet.h>
#include "TcpSrvCom.h"
#include "TcpSrvChannel.h"

//#include <sstream>

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CTcpServerCom".

CTcpServerCom::CTcpServerCom()
{
    m_pListenSocket = NULL;
	m_nCurChannel = -1;
    for (int i = 0; i < MAX_CLIENTS_COUNT; i++)
    m_pChannels[i] = NULL;
}

CTcpServerCom::~CTcpServerCom()
{
	for (int i = 0; i < MAX_CLIENTS_COUNT; i++)
	{
		if (m_pChannels[i] != NULL)
		{
			if (m_pChannels[i]->m_pSocket != NULL)
         {
//            m_pChannels[i]->m_pSocket->ShutDown();
//            m_pChannels[i]->m_pSocket->Detach();
				delete m_pChannels[i]->m_pSocket;
                m_pChannels[i]->m_pSocket = NULL;
         }
            if (m_pChannels[i] != NULL)
            {
                delete m_pChannels[i];
                m_pChannels[i] = NULL;
            }
		}
	}

#if !defined _WIN32_WCE

	if (m_pListenSocket != NULL)
   {
//      m_pListenSocket->ShutDown();
//      m_pListenSocket->Detach();

        delete m_pListenSocket;
        m_pListenSocket = NULL;
   }
#endif
}

BOOL CTcpServerCom::Init(UINT uPort)
{
	// STEP 2: Create The Listening Socket
	if ((m_pListenSocket = new CListeningSocket(this)) == NULL)
		return FALSE;

	if (m_pListenSocket->Create(uPort))
	{
        //treat by sfe1012
//		if (!m_pListenSocket->Listen())
//			return FALSE;
	}
   else
      return FALSE;

	return TRUE;

}

//
//   Create a new channel. (This function is designed so that derived classes
//   can have an opptunity to create the channel in different manner).
//
int CTcpServerCom::CreateChannel(CHAR* szIpAddr, int nChannel)
{
   if (nChannel >= 0)
   {
      if (m_pChannels[nChannel] != NULL)    // Channel already opened
         return -1;

      // Create a new channel
      CTcpServerChannel* pChannel = new CTcpServerChannel(this, nChannel, szIpAddr);
      if (pChannel != NULL)
      {
         m_pChannels[nChannel] = pChannel;
         return nChannel;
      }
      else
         return -1;
   }
   else  // nChannel < 0
   {
      for (int i = 0; i < MAX_CLIENTS_COUNT; i++)
      {
         if (m_pChannels[i] == NULL)
            return CreateChannel(szIpAddr, i);
      }
      return -1;
   }
}

//
//   Select a new channel as the current channel.
//
//   Return:
//     TRUE  - if the selection is correct
//     FALSE - if the selection is wrong
//
BOOL CTcpServerCom::SelectChannel(int nChannel)
{
	// Check channel range, check wether the channel is active
   if (nChannel >= MAX_CLIENTS_COUNT || m_pChannels[nChannel] == NULL)
      return FALSE;

	m_nCurChannel = nChannel;
	return TRUE;
}

//
//   RxReady: Test whether the client channel is ready for reading.
//
BOOL CTcpServerCom::RxReady(int nChannel)
{
	if (nChannel >= 0)
		return (m_pChannels[nChannel]->m_nRecvCount > 0);
	else
	{
		if (m_nCurChannel < 0)
         return FALSE;
      else
   		return RxReady(m_nCurChannel);
	}
}

//
//   Get the IP address of the specified client.
//
CHAR* CTcpServerCom::GetIpAddr(int nChannel)
{
	return m_pChannels[nChannel]->m_szIpAddr;
}

void CTcpServerCom::DoSend()
{
	m_pChannels[m_nCurChannel]->DoSend();
}

BOOL CTcpServerCom::AcceptConnect(CTcpSocket* pSocket, SOCKADDR_IN& Peer)
{
	// Find out which client this socket belongs to
	int nChannel = GetClientID(Peer);

	if (nChannel < 0)
	{
      nChannel = CreateChannel(GetPeerIP(Peer));
      if (nChannel < 0)
         return FALSE;
	}

	// Attaches the client to the socket
	m_pChannels[nChannel]->AttachSocket(pSocket);
	OnConnectChannel(nChannel);

   return TRUE;
}

//
//   Close the channel
//
BOOL CTcpServerCom::CloseChannel(int nChannel)
{
   m_CritSection.Lock();
   if(m_pChannels[nChannel] != NULL )
   {
       delete m_pChannels[nChannel];
       m_pChannels[nChannel] = NULL;
    }
   m_CritSection.Unlock();
   return TRUE;
}

///////////////////////////////////////////////////////////////////////////
//   Helper functions.

//
//   Get the client ID from its IP address.
//
int CTcpServerCom::GetClientID(CHAR* szIpAddr)
{
	for (int i = 0; i < MAX_CLIENTS_COUNT; i++)
   {
		if ((m_pChannels[i] != NULL) && !strcmp(szIpAddr, m_pChannels[i]->m_szIpAddr))
			return i;
   }
	return -1;      // Not found
}

//
//   GetClientID: Find the ID of the client from the given socket address.
//
int CTcpServerCom::GetClientID(SOCKADDR_IN& Peer)
{
	// Get the IP address string
	CHAR* szIpAddr = GetPeerIP(Peer);
	return GetClientID(szIpAddr);
}

//
//   GetClientID: Find ID of the client from the given channel pointer.
//
int CTcpServerCom::GetClientID(CTcpServerChannel* pChannel)
{
	if (pChannel == NULL)
      return -1;

	for (int i = 0; i < MAX_CLIENTS_COUNT; i++)
		if (m_pChannels[i] == pChannel)
			return i;

	return -1;
}

CHAR* CTcpServerCom::GetPeerIP(SOCKADDR_IN& Peer)
{
    //#if !defined _UNICODE
   strncpy(m_uchIpBuf, inet_ntoa(Peer.sin_addr), 16);
  //  mbstowcs(m_uchIpBuf, inet_ntoa(Peer.sin_addr), 16);
//#else
//   MultiByteToWideChar(936, 0, inet_ntoa(Peer.sin_addr), -1, m_uchIpBuf, 16);
//#endif
   return m_uchIpBuf;
}
