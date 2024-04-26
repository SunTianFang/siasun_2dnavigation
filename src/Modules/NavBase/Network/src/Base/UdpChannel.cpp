//                                - UDPCHANNEL.CPP -
//
//   Implementatin of class "CUdpChannel".
//
//   Author: Zhang Lei
//   Date:   2012. 9. 9
//

#include <stdafx.h>
#include "UdpChannel.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CCriticalSection CUdpChannel::m_CritSec;

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CUdpChannel".

CUdpChannel::CUdpChannel(CString strRemoteIp, UINT uRemotePort, BOOL bFramed) :
CFrameChannel(bFramed)
{
    m_strRemoteIp = strRemoteIp;
    m_uRemotePort = uRemotePort;
    m_pSocket = NULL;

    #ifdef USE_COM_POINTCOMM
        m_pVirtualSerialPort = new ZVirtualSerialPortClient(VSP_ID_VKP);
    #else
        m_pSocket = NULL;
    #endif

}

void CUdpChannel::SetSocket( CUdpSocket* pSocket )
{
#ifndef USE_COM_POINTCOMM
    m_pSocket = pSocket;
#endif
}

BOOL CUdpChannel::InUse()
{
#ifndef USE_COM_POINTCOMM
    return m_pSocket != NULL;
#else
    return TRUE;
#endif
}

int CUdpChannel::Send(UCHAR* pBuf, int nBufLen)
{
    int nRet = 0;

#ifdef USE_COM_POINTCOMM
    m_CritSec.Lock();
    nRet = m_pVirtualSerialPort->SendData((char*)pBuf,nBufLen);
    m_CritSec.Unlock();

    return nRet;
#else

#ifdef WINDOWS_PLATFORM_USING
    int nSentBytes = 0;

    m_CritSec.Lock();
    nSentBytes = sendto(m_pSocket->m_Socket, (char*)pBuf, nBufLen, 0, (SOCKADDR*) &m_remoteAddress, m_remoteAddrSz);
    if(nSentBytes == SOCKET_ERROR)
    {
        m_pSocket->m_errorCode = WSAGetLastError();
        m_CritSec.Unlock();

        return SOCKET_ERROR;
    }
    m_CritSec.Unlock();

    return nSentBytes;
#elif defined(LINUX_PLATFORM_USING)
    //CSocket
    m_CritSec.Lock();
    if(NULL != m_pSocket)
    {
        nRet = m_pSocket->SendTo(pBuf, nBufLen, m_uRemotePort, m_strRemoteIp.c_str());
    }
    m_CritSec.Unlock();

    return nRet;
#endif

#endif
}

// dq VISION
int CUdpChannel::Send_Cam(UCHAR* pBuf, int nBufLen)
{
    int nRet = 0;

#ifdef USE_COM_POINTCOMM
    m_CritSec.Lock();
    nRet = m_pVirtualSerialPort->SendData((char*)pBuf,nBufLen);
    m_CritSec.Unlock();

    return nRet;
#else

#ifdef WINDOWS_PLATFORM_USING
    int nSentBytes = 0;

    m_CritSec.Lock();
    nSentBytes = sendto(m_pSocket->m_Socket, (char*)pBuf, nBufLen, 0, (SOCKADDR*) &m_remoteAddress, m_remoteAddrSz);
    if(nSentBytes == SOCKET_ERROR)
    {
        m_pSocket->m_errorCode = WSAGetLastError();
        m_CritSec.Unlock();

        return SOCKET_ERROR;
    }
    m_CritSec.Unlock();

    return nSentBytes;
#elif defined(LINUX_PLATFORM_USING)
    //CSocket
    m_CritSec.Lock();
    if(NULL != m_pSocket)
    {
        USHORT  m_uCamPort = 6688;
        CString m_TempIp = "192.168.3.250";
        nRet = m_pSocket->SendTo(pBuf, nBufLen, m_uCamPort, m_strRemoteIp.c_str());
        //nRet = m_pSocket->SendTo(pBuf, nBufLen, m_uCamPort, m_TempIp.c_str());
    }
    m_CritSec.Unlock();

    return nRet;
#endif

#endif
}

int CUdpChannel::Receive(UCHAR* pBuf1, int nBufLen)
{
#ifdef USE_COM_POINTCOMM
    m_RxCritSection.Lock();

    UCHAR* pBuf = m_RBuf + m_nRecvCount;
    int nBufSize = sizeof(m_RBuf) - m_nRecvCount;

    if (nBufSize <= 0)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 1;

        m_RxCritSection.Unlock();
        return 0;
    }

    // Receive data to the buffer
    int nLen = 0;
    m_pVirtualSerialPort->RecvData((char*)pBuf,nLen);

    if (nLen < 0)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 2;

        m_RxCritSection.Unlock();
        return 0;
    }
    // Update the total count of bytes left in the buffer
    m_nRecvCount += nLen;

    m_RxCritSection.Unlock();
    return nLen;
#else
    return 0;
#endif
}

//
//
//
void CUdpChannel::SetRemoteIp( CString strHostIp )
{
    m_strRemoteIp = strHostIp;

    #ifdef WINDOWS_PLATFORM_USING
        m_strRemoteIp = strHostIp;
        _tcscpy(m_szIpAddr, m_strRemoteIp);

        char chStr[32];
        WideCharToMultiByte(936, 0, m_szIpAddr, -1, chStr, 32, NULL, NULL);

        memset(&m_remoteAddress, 0, sizeof(SOCKADDR_IN));
        m_remoteAddress.sin_family = AF_INET;
        m_remoteAddress.sin_port = htons(m_uRemotePort);
        m_remoteAddress.sin_addr.s_addr = inet_addr(chStr);

        m_remoteAddrSz = sizeof(SOCKADDR_IN);
    #endif
}


