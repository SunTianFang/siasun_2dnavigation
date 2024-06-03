//                       - SERVERCOM.CPP -
//
//   Implementatin of class "CUdpServerCom", server version.
//
//

#include "stdafx.h"
#include <stdio.h>
#include "UdpSrvCom.h"
#include "UdpChannel.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CUdpServerCom".

CUdpServerCom::CUdpServerCom(USHORT uRemotePort, BOOL bFramed, BOOL bAscii)
{
    m_nCurChannel = -1;
    m_bConnected = FALSE;
    for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
        m_pChannels[i] = NULL;

    m_pUdpSocket = NULL;
    m_RemotePort = uRemotePort;
    m_bFramed = bFramed;
    m_bAscii = bAscii;

    m_chTxMask = ((UCHAR)'C');
    m_chRxId = ((UCHAR)'T');
    m_chTxId = 1;
    m_chRxId = 1;

    memset(m_uchIpBuf, 0, 16*sizeof(TCHAR));
    memset(m_chIpStr, 0, 32*sizeof(char));
}

CUdpServerCom::~CUdpServerCom()
{
    for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
    {
        if (m_pChannels[i] != NULL)
        {
            /*if (m_pChannels[i]->m_pSocket != NULL)
            {
                delete m_pChannels[i]->m_pSocket;
                m_pChannels[i]->m_pSocket = NULL;
            }*/

            delete m_pChannels[i];
            m_pChannels[i] = NULL;
        }
    }
}

BOOL CUdpServerCom::Init(UINT uPort)
{
    return TRUE;
}

//
//   Create a new channel. (This function is designed so that derived classes
//   can have an opptunity to create the channel in different manner).
//
int CUdpServerCom::CreateChannel(TCHAR* szIpAddr, UINT uPort, int nChannel)
{
    if (nChannel >= 0)
    {
        if (m_pChannels[nChannel] != NULL)    // Channel already opened
            return -1;

        // Create a new channel
        CUdpChannel* pChannel = new CUdpChannel(szIpAddr, uPort, m_bFramed);
        if (pChannel != NULL)
        {
            pChannel->SetProtocolFormat(m_bFramed, m_bAscii);
            pChannel->SetProtoID(m_chTxId, m_chRxId);
            pChannel->SetProtoMask(m_chTxMask, m_chRxMask);
            m_pChannels[nChannel] = pChannel;

            return nChannel;
        }
        else
            return -1;
    }
    else  // nChannel < 0
    {
        for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
        {
            if (m_pChannels[i] == NULL)
                return CreateChannel(szIpAddr, uPort, i);
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
BOOL CUdpServerCom::SelectChannel(int nChannel)
{
    // Check channel range, check wether the channel is active
    if (nChannel >= MAX_UDP_CLIENTS_COUNT || m_pChannels[nChannel] == NULL)
        return FALSE;

    m_nCurChannel = nChannel;
    return TRUE;
}

//
//   RxReady: Test whether the client channel is ready for reading.
//
BOOL CUdpServerCom::RxReady(int nChannel)
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
char* CUdpServerCom::GetIpAddr(int nChannel)
{
    if(nChannel < 0 || nChannel >= MAX_UDP_CLIENTS_COUNT)
    {
        return NULL;
    }

   // WideCharToMultiByte(936, 0, m_pChannels[nChannel]->m_szIpAddr, -1, m_chIpStr, 32, NULL, NULL);
    return m_pChannels[nChannel]->m_szIpAddr;
}

void CUdpServerCom::DoSend()
{
    m_pChannels[m_nCurChannel]->DoSend();
}

BOOL CUdpServerCom::CreateCom(TCHAR* szIpAddr)
{
    // Find out which client this socket belongs to
    int nChannel = GetClientID(szIpAddr);

    if (nChannel < 0)
    {
        nChannel = CreateChannel(szIpAddr, m_RemotePort);
        if (nChannel < 0)
            return FALSE;
    }

    // Attaches the client to the socket
    m_pChannels[nChannel]->SetSocket(m_pUdpSocket);
    OnConnectChannel(nChannel);

    m_bConnected = TRUE;
    return TRUE;
}

//
//   Close the channel
//
BOOL CUdpServerCom::CloseChannel(int nChannel)
{
    m_CritSection.Lock();
    delete m_pChannels[nChannel];
    m_pChannels[nChannel] = NULL;
    m_CritSection.Unlock();
    return TRUE;
}

///////////////////////////////////////////////////////////////////////////
//   Helper functions.

//
//   Get the client ID from its IP address.
//
int CUdpServerCom::GetClientID(TCHAR* szIpAddr)
{
    for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
    {
        if ((m_pChannels[i] != NULL) && !strcmp(szIpAddr, m_pChannels[i]->m_strRemoteIp.c_str()))
            return i;
    }
    return -1;      // Not found
}

//
//   GetClientID: Find the ID of the client from the given socket address.
//
int CUdpServerCom::GetClientID(SOCKADDR_IN& Peer)
{
    // Get the IP address string
    TCHAR* szIpAddr = GetPeerIP(Peer);
    return GetClientID(szIpAddr);
}

//
//   GetClientID: Find ID of the client from the given channel pointer.
//
int CUdpServerCom::GetClientID(CUdpChannel* pChannel)
{
    if (pChannel == NULL)
        return -1;

    for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
        if (m_pChannels[i] == pChannel)
            return i;

    return -1;
}

//
//   Get the client ID from its IP address.
//
int CUdpServerCom::GetClientID(CString strRemoteIp)
{
    for (int i = 0; i < MAX_UDP_CLIENTS_COUNT; i++)
    {
        if ((m_pChannels[i] != NULL) && !strcmp(strRemoteIp.c_str(), m_pChannels[i]->m_strRemoteIp.c_str()))
            return i;
    }
    return -1;      // Not found
}

TCHAR* CUdpServerCom::GetPeerIP(SOCKADDR_IN& Peer)
{
#if !defined _UNICODE
    strncpy(m_uchIpBuf, inet_ntoa(Peer.sin_addr), 16);
#else
    MultiByteToWideChar(936, 0, inet_ntoa(Peer.sin_addr), -1, m_uchIpBuf, 16);
#endif
    return m_uchIpBuf;
}


//
//   RxReady: Test whether the client channel is ready for reading.
//
int CUdpServerCom::GetRecvCount(int nChannel)
{
    if (nChannel >= 0)
        return (m_pChannels[nChannel]->m_nRecvCount);
    else
    {
        if (m_nCurChannel < 0)
            return 0;
        else
            return GetRecvCount(m_nCurChannel);
    }
}

BOOL CUdpServerCom::RecvFromBuf(CString strRemoteIp, UCHAR* pBuf, int nLen)
{
    // Find out which client this socket belongs to
    int nChannel = GetClientID(strRemoteIp);

    if (nChannel < 0)
        return FALSE;

    CUdpChannel* pChannel = NULL;
    if (m_pChannels[nChannel] != NULL)
    {
        pChannel = (CUdpChannel*)(m_pChannels[nChannel]->GetChannelObject());
    }

    if (pChannel != NULL)
    {
        pChannel->ReceiveFromBufferEx(pBuf, nLen);
    }

    return TRUE;
}
