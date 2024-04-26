//                    - AGVUDPSOCK.CPP -
//
//   Implementation of class "CAgvUdpSocket".
//
//   Author: Zhanglei
//   Date:   2012. 10. 7
//

#include "stdafx.h"
#include "AgvUdpSock.h"
#include "UdpChannel.h"
#include "UdpSrvCom.h"
#include "Tools.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CAgvUdpSocket".

CAgvUdpSocket::CAgvUdpSocket() 
{
    m_pChannel = NULL;
    m_pRLPChannel = NULL;
    m_pUdpServerCom = NULL;
    m_uLayoutPort = 0;
    m_uRLClntPort = 0;
    m_uRLSrvPort = 0;
    m_strRLSrvIp = ' ';
}

/*
//
//   Create the UDP communication object.
//
BOOL CAgvUdpSocket::Create(CString strMainHostIp, UINT uHostPort, UINT uMaintainPort, UINT uLocalPort,
                                    CString strBackupHostIp)
{
    m_strHostIp[0] = strMainHostIp;
    m_strHostIp[1] = strBackupHostIp;

   m_uHostPort = uHostPort;
   m_uMaintainPort = uMaintainPort;
   m_nHostAck = -1;
    m_dwLastEchoTime = GetTickCount();
    m_pChannel = NULL;

   return CUdpSocket::Create(uLocalPort);
}
*/

//
//   生成Socket对象，并设置通讯端口。
//
BOOL CAgvUdpSocket::Create(UINT uLocalPort, UINT uHostPort, UINT uVKIPort)
{
    m_uHostPort = uHostPort;
    m_uMaintainPort = uVKIPort;
    m_nHostAck = -1;
    m_dwLastEchoTime = GetTickCount();
    m_pChannel = NULL;

    return CUdpSocket::Create(uLocalPort);
}

//
//   设置上位机(TC)的IP地址。
//
void CAgvUdpSocket::SetHostIp(CString strMainHostIp, CString strBackupHostIp)
{
    m_strHostIp[0] = strMainHostIp;
    m_strHostIp[1] = strBackupHostIp;
}

//
//   Receive UDP packets.
//
void CAgvUdpSocket::DoReceive()
{
    CUdpMsg Msg;
	 // dq VISION 6688
    UINT m_CamPort =6688;
    CUdpSocket::DoReceive();

#ifdef WINDOWS_PLATFORM_USING
    int iCount = m_MsgList.GetCount();
#elif defined(LINUX_PLATFORM_USING)
    int iCount = m_MsgList.size();
    //		Msg = m_MsgList.front();
    //		m_MsgList.pop_front();
#endif

    while (iCount > 0)
    {
#ifdef WINDOWS_PLATFORM_USING
        // Retrieve a message from the head
        Msg = m_MsgList.RemoveHead();
#elif defined(LINUX_PLATFORM_USING)
        Msg = m_MsgList.front();
        m_MsgList.pop_front();
        iCount--;
#endif
        // If the message is from the stationary computer, update the "Last echo time"
        if (Msg.uPort == m_uHostPort)
        {
            if (Msg.strSockAddr == m_strHostIp[0])
            {
                m_nHostAck = 0;
                m_dwLastEchoTime = GetTickCount();
            }
            else if (Msg.strSockAddr == m_strHostIp[1])
            {
                m_nHostAck = 1;
                m_dwLastEchoTime = GetTickCount();
            }
        }
        // If the message is from a maintainence tool
        else if (Msg.uPort == m_uMaintainPort)
        {
            ASSERT(m_pChannel != NULL);

            CUdpChannel* pChannel = (CUdpChannel*)(m_pChannel->GetChannelObject());
            pChannel->ReceiveFromBuffer(Msg.uchMsg, Msg.nLen);
            pChannel->SetRemoteIp(Msg.strSockAddr);
        }
        // If the message is from Layout wizard
        else if (Msg.uPort == m_uLayoutPort)
        {
#ifdef WINDOWS_PLATFORM_USING
            int sentBytes = 0;
            TCHAR szIpAddr[16];
            char chStr[32];
            SOCKADDR_IN	remoteAddress;
            int	remoteAddrSz;

            _tcscpy(szIpAddr, Msg.strSockAddr);
            WideCharToMultiByte(936, 0, szIpAddr, -1, chStr, 32, NULL, NULL);

            memset(&remoteAddress, 0, sizeof(SOCKADDR_IN));
            remoteAddress.sin_family = AF_INET;
            remoteAddress.sin_port = htons(Msg.uPort);
            remoteAddress.sin_addr.s_addr = inet_addr(chStr);

            remoteAddrSz = sizeof(SOCKADDR_IN);
            sentBytes = sendto(m_Socket, (char*)Msg.uchMsg, Msg.nLen, 0, (SOCKADDR*) &remoteAddress, remoteAddrSz);

#elif defined(LINUX_PLATFORM_USING)
            //CSocket
            SendTo(Msg.uchMsg, Msg.nLen, Msg.uPort, Msg.strSockAddr.c_str());
#endif
        }
        else if ( Msg.uPort == m_uRLSrvPort)
        {
            if (Msg.strSockAddr == m_strRLSrvIp)
            {
                ASSERT(m_pRLPChannel != NULL);
                CUdpChannel* pChannel = NULL;
                if (m_pRLPChannel != NULL)
                {
                    pChannel = (CUdpChannel*)(m_pRLPChannel->GetChannelObject());
                }
                if (pChannel != NULL)
                {
                    pChannel->ReceiveFromBufferEx(Msg.uchMsg, Msg.nLen);
                    //pChannel->SetRemoteIp(Msg.strSockAddr);
                    SetEvent(pChannel->m_hRecvCalling);
                }
            }
        }

        // dq VISION get msg from Cam
        else if ( Msg.uPort == m_CamPort)
        {
            if (Msg.strSockAddr == m_strRLSrvIp)
            {
                ASSERT(m_pRLPChannel != NULL);
                CUdpChannel* pChannel = NULL;
                if (m_pRLPChannel != NULL)
                {
                    pChannel = (CUdpChannel*)(m_pRLPChannel->GetChannelObject());
                }
                if (pChannel != NULL)
                {
                    pChannel->ReceiveFromBufferEx_cam(Msg.uchMsg, Msg.nLen);
                    //pChannel->SetRemoteIp(Msg.strSockAddr);
                    SetEvent(pChannel->m_hRecvCalling);
                }
            }
        }

        else if ( Msg.uPort == m_uRLClntPort)
        {
            ASSERT(m_pUdpServerCom != NULL);
            if (m_pUdpServerCom != NULL)
            {
                m_pUdpServerCom->RecvFromBuf(Msg.strSockAddr, Msg.uchMsg, Msg.nLen);

                SetEvent(m_pUdpServerCom->m_hRecvCalling);
            }
        }
    }

    //
    //    if (m_pUdpServerCom != NULL)
    //    {
    //        SetEvent(m_pUdpServerCom->m_hRecvCalling);
    //    }
}

//
//   Send a datagram to the host compter to check whether it is alive.
//
void CAgvUdpSocket::PingHost()
{
    m_nHostAck = -1;

    for (int i = 0; i < 2; i++)
    {
#ifdef WINDOWS_PLATFORM_USING// If the length of the host IP is longer than 0, it is a valid host IP address
        int sentBytes = 0;
        char pBuf[10] = "Ping Host";
        int nBufLen = 10;
        TCHAR szIpAddr[16];
        char chStr[32];
        SOCKADDR_IN	remoteAddress;
        int	remoteAddrSz;

        _tcscpy(szIpAddr, m_strHostIp[i]);
        WideCharToMultiByte(936, 0, szIpAddr, -1, chStr, 32, NULL, NULL);

        memset(&remoteAddress, 0, sizeof(SOCKADDR_IN));
        remoteAddress.sin_family = AF_INET;
        remoteAddress.sin_port = htons(m_uHostPort);
        remoteAddress.sin_addr.s_addr = inet_addr(chStr);

        remoteAddrSz = sizeof(SOCKADDR_IN);
        sentBytes = sendto(m_Socket, pBuf, nBufLen, 0, (SOCKADDR*) &remoteAddress, remoteAddrSz);
#elif defined(LINUX_PLATFORM_USING)
        if (m_strHostIp[i].length() > 0)
        {
            //std::cout << m_strHostIp[i].c_str() << "::" << m_uHostPort << std::endl;
            SendTo("Ping Host", 10, m_uHostPort, m_strHostIp[i].c_str());
        }

#endif
    }
}

//
//   Check whether an echo has been received from the host computer.
//
int CAgvUdpSocket::GetPingHostAck(DWORD dwTimeOut)
{
    if ((m_nHostAck >= 0) && (GetTickCount() - m_dwLastEchoTime < dwTimeOut))
        return m_nHostAck;
    else
        return -1;       // No ACK
}
