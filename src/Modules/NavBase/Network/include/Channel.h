//                        - CHANNEL.H -
//
//   The interface of class "CChannel" (Client version).
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#ifndef __CChannel
#define __CChannel

#include "Afxmt.h"
#include "ZTypes.h"
#include"LinuxSetting.h"

#define COM_BUFSIZE          8192

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CDataBuf".
class DllExport CDataBuf
{
public:
    CHAR*  m_pBuf;
    USHORT m_uSize;

public:
    CDataBuf(USHORT uSize, CHAR* pBuf)
    {
        m_pBuf = pBuf;
        m_uSize = uSize;
    }
};

////////////////////////////////////////////////////////////////////////
//   The interface of class "CChannel".
class DllExport CChannel
{
public:
    UCHAR        m_SBuf[COM_BUFSIZE];  // Output buffer
    UCHAR        m_RBuf[COM_BUFSIZE];  // Input buffer
    UCHAR        m_TempBuf[COM_BUFSIZE]; // buffer for temporary storage
    UCHAR*       m_pSend;              // Pointer to the current send position
    UCHAR*       m_pRecv;              // Pointer to the current receive position

    int          m_nRecvCount;         // Count of bytes left in receive buffer
    // dq VISION 区分cam&车体收到信息
    int          m_nRecFromCam = -1;
    TCHAR        m_szIpAddr[16];       // Host IP address

    USHORT       m_uTxSum;           // Sum of transmitted data
    USHORT       m_uRxSum;           // Sum of received data
    USHORT       m_uTxLen;
    USHORT       m_uRxLen;

    // dq VISION
    UCHAR           m_uchRxXor;
    UCHAR           m_uchRxLenLANXIN;

    int			m_nErrorCode;
    int          m_nRecvCountTemp;

    CCriticalSection m_RxCritSection;

public:
    // The constructor
    CChannel();

    virtual BOOL CreateSocket() = 0;

    // Detach the socket with the channel
    virtual void DetachSocket() = 0;

    CChannel* GetChannelObject() {return this;}

    virtual BOOL InUse() = 0;

    // Test whether the client channel is ready for reading
    BOOL RxReady() {return (m_nRecvCount > 0);}

	// dq VISION
    virtual int Send(UCHAR* pBuf, int nBufLen) = 0;
    virtual int Send_Cam(UCHAR* pBuf, int nBufLen) = 0;

    virtual int Receive(UCHAR* pBuf, int nBufLen) = 0;

    // Send out all the data in the output buffer
	// dq VISION
    virtual BOOL DoSend();
    virtual BOOL DoSend_Cam();
    // Receive all the data available to the input buffer
    virtual void DoReceive();

    virtual void PutString(TCHAR* str, int nLen);
    virtual int GetString(TCHAR* str, int nMaxLen);

    // 将一段数据直接加入到接收缓冲区中。
    int ReceiveFromBuffer(UCHAR* pBuf, int nLen);

    void ReceiveFromBufferEx(UCHAR* pBuf, int nLen);
    void ReceiveFromBufferEx_cam(UCHAR* pBuf, int nLen);
    int GetRecvCount();
    bool GetRecvFromCam();
};

///////////////////////////////////////////////////////////////////////////////
//   Implementation of the 2 overloaded operators: << , >> 

template <class T> 
DllExport CChannel& operator << (CChannel& Channel, T Obj)
{
    UCHAR uch;
    UCHAR* p = (UCHAR*)&Obj;

    for (int i = 0; i < sizeof(T); i++)
    {
        if (Channel.m_pSend - Channel.m_SBuf >= COM_BUFSIZE)
        {
            ASSERT(FALSE);
            return Channel;
        }

        uch = *p++;
        *Channel.m_pSend++ = uch;
        Channel.m_uTxSum += uch;
        Channel.m_uTxLen++;
    }

    return Channel;
}

template <class T> 
DllExport CChannel& operator >> (CChannel& Channel, T& Obj)
{
    UCHAR uch;
    UCHAR* p = (UCHAR*)&Obj;

    Channel.m_RxCritSection.Lock();

    for (int i = 0; i < sizeof(T); i++)
    {
        if (Channel.m_nRecvCount == 0)
        {
            Channel.m_RxCritSection.Unlock();
            return Channel;
        }

        uch = *Channel.m_pRecv++;
        *p++ = uch;
        Channel.m_uRxSum += uch;
        Channel.m_uRxLen++;
        Channel.m_nRecvCount--;
    }

    // Make sure that the buffer is not overflow
    if (Channel.m_pRecv - Channel.m_RBuf >= COM_BUFSIZE || Channel.m_nRecvCount <= 0)
    {
        Channel.m_nRecvCount = 0;
    }
    if (Channel.m_nRecvCount)
    {
        for (int nIndex=0; nIndex<Channel.m_nRecvCount; ++nIndex)
        {
            Channel.m_RBuf[nIndex] = Channel.m_RBuf[nIndex+sizeof(T)];
        }
    }
    Channel.m_pRecv = Channel.m_RBuf;

    Channel.m_RxCritSection.Unlock();

    return Channel;
}
#endif
