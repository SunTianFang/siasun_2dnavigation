//                     - FRMCHAN.H -
//
//   The interface of class "CFrameChannel".
//
//   Author: Zhang Lei
//   Date:   2012. 9. 9
//

#ifndef __CFrameChannel
#define __CFrameChannel

#include "Channel.h"

/////////////////////////////////////////////////////////////////////////////
//   The interface of class "CFrameChannel".
class DllExport CFrameChannel : public CChannel
{
private:
    UCHAR* m_pTxLenBuf;
    USHORT m_uRxLen0;            // Rx data length (data extracted from the frame)
    BOOL   m_bFramed;
    BOOL   m_bAscii;				// The data is coded in ASCII format

    UCHAR* m_pHeaderSumBuf;
    USHORT m_uHeaderSum;
    UCHAR  m_uLocalMask;
    UCHAR  m_uRemoteMask;

    UCHAR* m_pCalcHeadCRCBuf;
    UCHAR* m_pCalcBodyCRCBuf;
    UCHAR  m_uLocalID;			// AGV NO.
    UCHAR  m_uRemoteID;			// TC send to AGV NO.
    USHORT m_uRxBodyCRCSum;			// CRC SUM
    std::string m_ProtoVersion; // 协议格式版本

public:

    CFrameChannel(BOOL bFramed = TRUE, BOOL bAscii = FALSE);

    void InitTxPacket(USHORT uTxLen = 0);
    void FinishTxPacket();
    BOOL GetRxHeader();
    BOOL CheckRxEnd();
    BOOL IsAscii(){ return m_bAscii;}
    void SetProtocolFormat(BOOL bFramed = FALSE, BOOL bAscii = FALSE);

    void SetProtoMask(UCHAR localMask, UCHAR remoteMask)
    {
        m_uLocalMask = localMask;
        m_uRemoteMask = remoteMask;
    }

    void SetProtoVersion(std::string strVersion) {m_ProtoVersion = strVersion;}

    void SetProtoID(UCHAR localID, UCHAR remoteID)
    {
        m_uLocalID = localID;
        m_uRemoteID = remoteID;
    }

    // Overloaded operators
    CFrameChannel& operator << (char ch);
    CFrameChannel& operator >> (char& ch);

#ifdef WINDOWS_PLATFORM_USING
    virtual void PutString(TCHAR* str, int nLen);
    virtual void PutString(TCHAR* str);
    virtual int GetString(TCHAR* str, int nMaxLen);
#elif defined(LINUX_PLATFORM_USING)
    virtual void PutString(UCHAR* str, int nLen);
    virtual void PutString(UCHAR* str);
    virtual int GetString(UCHAR* str, int nMaxLen);
#endif
    BOOL Preprocess();

    USHORT CRC16TableFast(UCHAR *ptr, USHORT len);

    // dq VISION 校验从CAM收到POS信息头尾
    BOOL GetRxHeaderLANXIN();
    BOOL CheckRxEndLANXIN();
};


template <class T> 
DllExport CFrameChannel& operator << (CFrameChannel& Proto, T Obj)
{
    char *p = (char*)(&Obj);

    for (int i = 0; i < sizeof(T); i++)
        Proto << p[i];

    return Proto;
}

template <class T> 
DllExport CFrameChannel& operator >> (CFrameChannel& Proto, T& Obj)
{
    char *p = (char*)(&Obj);

    for (int i = 0; i < sizeof(T); i++)
        Proto >> p[i];

    return Proto;
}

#endif
