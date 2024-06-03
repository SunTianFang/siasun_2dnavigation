//                                - CHANNEL.CPP -
//
//   Implementatin of class "Channel".
//
//   Author: Zhang Lei
//   Date:   2012. 9. 9
//

#include <stdafx.h>
#include "Channel.h"
#include "ZTypes.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
#include"Debug.h"

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CChannel".

CChannel::CChannel()
{
    m_nRecvCountTemp = 0;
    m_nErrorCode = 0;
    m_pSend = m_SBuf;
    m_pRecv = m_RBuf;
    m_uTxSum = m_uRxSum = 0;
    m_nRecvCount = 0;
    m_uTxLen = m_uRxLen = 0;
    // dq VISION
    m_uchRxXor = 0;
}

//
//   Send all the data in the output buffer through the data socket.
//
BOOL CChannel::DoSend()
{
    int nLen = m_pSend - m_SBuf;

    if (nLen <= 0 || nLen > COM_BUFSIZE)
    {
        m_pSend = m_SBuf;
        return FALSE;
    }

    m_pSend = m_SBuf;
    return (Send(m_SBuf, nLen) != SOCKET_ERROR);
}

// dq VISION
BOOL CChannel::DoSend_Cam()
{
    int nLen = m_pSend - m_SBuf;

    if (nLen <= 0 || nLen > COM_BUFSIZE)
    {
        m_pSend = m_SBuf;
        return FALSE;
    }

    m_pSend = m_SBuf;
    return (Send_Cam(m_SBuf, nLen) != SOCKET_ERROR);
}
//
//   Receive all the data available in the data socket into the input buffer.
//
void CChannel::DoReceive()
{
    std::cout<<"CChannel::DoReceive()"<<std::endl;
    m_RxCritSection.Lock();

    UCHAR* pBuf = m_RBuf + m_nRecvCount;
    int nBufSize = sizeof(m_RBuf) - m_nRecvCount;

    if (nBufSize <= 0)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 1;

        m_RxCritSection.Unlock();
        return;
    }

    // Receive data to the buffer
    int nLen = Receive(pBuf, nBufSize);
    if (nLen == SOCKET_ERROR)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 2;

        m_RxCritSection.Unlock();
        return;
    }
    // Update the total count of bytes left in the buffer
    m_nRecvCount += nLen;

    m_RxCritSection.Unlock();
}

void CChannel::PutString(TCHAR* str, int nLen)
{
    USHORT uLen = _tcslen(str);
    *this << uLen;

    for (USHORT i = 0; i < uLen; i++)
        *this << str[i];
}

int CChannel::GetString(TCHAR* str, int nMaxLen)
{
    USHORT uLen, i;
    *this >> uLen;
    if ((int)uLen >= nMaxLen)
        uLen = nMaxLen-1;

    for (i = 0; i < uLen; i++)
        *this >> str[i];

    return (int)uLen;
}

//
//   将一段数据直接加入到接收缓冲区中。
//
int CChannel::ReceiveFromBuffer(UCHAR* pBuf, int nLen)
{
    int i;

    m_RxCritSection.Lock();

    for (i = 0; i < nLen; i++)
    {
        m_pRecv[i] = *pBuf++;
        if (++m_uRxLen == COM_BUFSIZE)
            break;
        else
        {
            // Update the total count of bytes left in the buffer
            m_nRecvCount++;
        }
    }

    m_RxCritSection.Unlock();
    return i;
}

///
///
///
void CChannel::ReceiveFromBufferEx(UCHAR* pBuf, int nLen)
{
    int i, nRxLen;

    m_RxCritSection.Lock();

    UCHAR* pRxBuf = m_RBuf + m_nRecvCount;
    int nBufSize = sizeof(m_RBuf) - m_nRecvCount;

    if (nBufSize <= 0 || nLen <= 0)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 1;

        m_RxCritSection.Unlock();
        return;
    }

    // Receive data to the buffer
    if (nBufSize >= nLen)
    {
        nRxLen = nLen;
    }
    else
    {
        nRxLen = nBufSize;
    }

    for (i = 0; i < nRxLen; i++)
    {
        *pRxBuf++ = *pBuf++;
    }

    // Update the total count of bytes left in the buffer
    m_nRecvCount += nRxLen;
    m_nRecFromCam = 0;
    m_RxCritSection.Unlock();
}


//dq VISION
void CChannel::ReceiveFromBufferEx_cam(UCHAR* pBuf, int nLen)
{
    int i, nRxLen;

    m_RxCritSection.Lock();

    UCHAR* pRxBuf = m_RBuf + m_nRecvCount;
    int nBufSize = sizeof(m_RBuf) - m_nRecvCount;

    if (nBufSize <= 0 || nLen <= 0)
    {
        m_nRecvCountTemp = m_nRecvCount;
        m_nErrorCode = 1;

        m_RxCritSection.Unlock();
        return;
    }

    // Receive data to the buffer
    if (nBufSize >= nLen)
    {
        nRxLen = nLen;
    }
    else
    {
        nRxLen = nBufSize;
    }

    for (i = 0; i < nRxLen; i++)
    {
        *pRxBuf++ = *pBuf++;
    }

    // Update the total count of bytes left in the buffer
    m_nRecvCount += nRxLen;
    m_nRecFromCam = 1;
    m_RxCritSection.Unlock();
}

int CChannel::GetRecvCount()
{
    int uCount;

    m_RxCritSection.Lock();
    uCount = m_nRecvCount;
    m_RxCritSection.Unlock();

    return uCount;
}

bool CChannel::GetRecvFromCam()
{
    bool uRecvFromCam;

    m_RxCritSection.Lock();
    uRecvFromCam = m_nRecFromCam;
    m_RxCritSection.Unlock();

    return uRecvFromCam;
}
