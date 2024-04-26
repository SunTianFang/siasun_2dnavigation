//                        - FRMCHAN.CPP -
//
//   Implementatin of class "CFrameChannel".
//
//   Author: Zhang Lei
//   Date:   2012. 9. 9
//

#include "stdafx.h"
#include "FrmChan.h"
#include "Tools.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#ifdef AGC_MODE
#ifdef USE_RFID_TOOL
#define STX                ((UCHAR)'\2')       // Start of Text character
#define ETX                ((UCHAR)'\3')       // End of Text character
#else
#define STX                ((UCHAR)'\1')       // Start of Text character
#define ETX                ((UCHAR)'\2')       // End of Text character
#endif
#else
#define STX                ((UCHAR)'\1')       // Start of Text character
#define ETX                ((UCHAR)'\2')       // End of Text character
#endif

#define TCP_STX             ((UCHAR)'\2')       // Start of Text character in TCP/IP ASCII protocol
#define TCP_ETX             ((UCHAR)'\3')       // End of Text character in TCP/IP ASCII protocol
#define SERVER_ID			((UCHAR)'T')
#define CLIENT_ID			((UCHAR)'C')
#define NET_VERSION			((UCHAR)'\1')

#define ASCII_MSG_MIN_LEN		 16
#define ASCII_MSG_MAX_LEN		 1024

#define NET_PROTO_VER_1_0   "1.0"   // 基础版本协议
#define NET_PROTO_VER_2_0   "2.0"   // 新版本协议

USHORT CRCTalbeABS[] = {
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
        0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CFrameChannel".

CFrameChannel::CFrameChannel(BOOL bFramed, BOOL bAscii)
{
    m_bFramed = bFramed;
    m_pTxLenBuf = NULL;
    m_uRxLen0 = 0;
    m_bAscii = bAscii;
    m_pHeaderSumBuf = NULL;
    m_pCalcHeadCRCBuf = NULL;
    m_pCalcBodyCRCBuf = NULL;

    m_uLocalMask = CLIENT_ID;
    m_uRemoteMask = SERVER_ID;
    m_uLocalID = 1;
    m_uRemoteID = 1;
    m_uRxBodyCRCSum = 0;
    m_ProtoVersion = NET_PROTO_VER_1_0;
}

void CFrameChannel::InitTxPacket(USHORT uTxLen)
{
    if (m_bFramed)
    {
        m_uTxSum = 0;
        m_uTxLen = 0;
        (*GetChannelObject()) << STX;

        m_pTxLenBuf = m_pSend;
        for (int i = 0; i < 4; i++)
            (*GetChannelObject()) << (UCHAR)0;               // Leave room for TxLen
    }
    else if (m_bAscii)
    {
        m_uTxSum = 0;
		m_uHeaderSum = 0;
        m_uTxLen = 0;

        (*GetChannelObject())<< TCP_STX;	// STX
        *this << m_uLocalMask;	// 'C'
        m_pCalcHeadCRCBuf = m_pSend;

        if(m_ProtoVersion == NET_PROTO_VER_2_0){
            *this << m_uLocalID;
        }

		m_pTxLenBuf = m_pSend;
        for (int i = 0; i < 4; i++)
			(*GetChannelObject()) << (UCHAR)0;               // Leave room for TxLen

		// Begin to calculate length and sum
        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            *this << NET_VERSION;
        }

		m_pHeaderSumBuf = m_pSend;
		m_uHeaderSum = m_uTxSum;
        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            for (int i = 0; i < 2; i++)
                (*GetChannelObject()) << (UCHAR)0;               // Leave room for HeaderSum
        }
        else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            for (int i = 0; i < 4; i++)
                (*GetChannelObject()) << (UCHAR)0;               // Leave room for HeaderSum
        }

        m_pCalcBodyCRCBuf = m_pSend;
		m_uTxSum = 0;
		m_uTxLen = 0;
	}
}

void CFrameChannel::FinishTxPacket()
{
    if (m_bFramed)
    {
        UCHAR* p = m_pSend;
        m_pSend = m_pTxLenBuf;

        *this << m_uTxLen;

        // Restore old send buffer pointer
        m_pSend = p;

        // Send the checksum and ETX
        *this << m_uTxSum;
        (*GetChannelObject()) << ETX;
    }
    else if (m_bAscii)
    {
        USHORT uTxSum = 0;
        USHORT uTxSumTemp = 0;
        USHORT uTxSumNew = 0;
        USHORT uTxLen = 0;
        USHORT uTxLenTemp = 0;
        // Send the checksum
        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            uTxSum = m_uTxSum;
            *this << (UCHAR)uTxSum;
            uTxLen = m_uTxLen;
        }
        else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            uTxLen = m_uTxLen;
            uTxSumNew = CRC16TableFast(m_pCalcBodyCRCBuf, m_uTxLen);
            *this << uTxSumNew;
        }

        UCHAR* p = m_pSend;
        m_pSend = m_pTxLenBuf;

        uTxSumTemp = m_uTxSum;
        uTxLenTemp = m_uTxLen;
		m_uTxSum = m_uHeaderSum;

        *this << uTxLen;

		m_uHeaderSum = m_uTxSum;
		m_pSend = m_pHeaderSumBuf;

        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            *this << (UCHAR)m_uHeaderSum;
        }
        else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            uTxSumNew = CRC16TableFast(m_pCalcHeadCRCBuf, 6);
            *this << uTxSumNew;
        }

		// Restore old send buffer pointer
		m_pSend = p;
        m_uTxSum = uTxSumTemp;
        m_uTxLen = uTxLenTemp;

		// Send the ETX
		(*GetChannelObject())<<TCP_ETX;
	}
}

BOOL CFrameChannel::GetRxHeader()
{
    if (m_bFramed)
    {
        char uch;

        m_uRxSum = 0;
        m_uRxLen = 0;
        (*GetChannelObject()) >> uch;
        if (uch != STX)
            return FALSE;
        else
        {
            m_uRxSum = STX;
            *this >> m_uRxLen0;
            return TRUE;
        }
    }
    else if (m_bAscii)
    {
        UCHAR uch = 0, uchMark = 0;
        UCHAR uchID = 0;
        UCHAR uchVersion = 0, uchVersionTemp = 0; //add By sfe1012
        UCHAR uchHeaderCheckSum0 = 0, uchHeaderCheckSum = 0;

        //m_uRxSum = 0;
        //m_uRxLen = 0;

        //*this >> uch;
        m_uRxSum = 0;
        (*GetChannelObject()) >> uch;

        if (uch != TCP_STX)
            return FALSE;
        else
        {
            *this >> uchMark;
            if (uchMark != m_uRemoteMask)
                return FALSE;

            USHORT uRxSumNew = 0;
            USHORT uHeaderCheckSumNew = 0;
            UCHAR* ptrRecvBuf = NULL;
            if (m_ProtoVersion == NET_PROTO_VER_2_0) {
                // 计算消息头校验和
                m_RxCritSection.Lock();
                ptrRecvBuf = m_pRecv;
                if(ptrRecvBuf == NULL || m_nRecvCount < 6){
                    m_RxCritSection.Unlock();
                    return FALSE;
                }
                uRxSumNew = CRC16TableFast(ptrRecvBuf, 6);
                m_RxCritSection.Unlock();

                *this >> uchID;
                if (uchID != m_uRemoteID){
                    return FALSE;
                }
            }

            m_uRxLen0 = 0;
            *this >> m_uRxLen0;

            if(m_ProtoVersion == NET_PROTO_VER_1_0){
                *this >> uchVersion;
                uchVersionTemp = uchVersion;// sfe add

                uchHeaderCheckSum = m_uRxSum;
                if(uchVersion != NET_VERSION)
                {
                    //				uchVersion = uchVersion ;
                    uchVersion = uchVersionTemp;
                }

                *this >> uchHeaderCheckSum0;
                if(uchHeaderCheckSum0 != uchHeaderCheckSum)
                    return FALSE;
            }
            else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
                *this >> uHeaderCheckSumNew;
                if(uRxSumNew != uHeaderCheckSumNew){
                    return FALSE;
                }
            }

            m_uRxSum = 0;//从消息头标示开始计算校验和
            m_uRxLen = 0;//从消息头标示开始计算消息长度

            // 计算消息体校验和
            if (m_ProtoVersion == NET_PROTO_VER_2_0) {
                m_RxCritSection.Lock();
                m_uRxBodyCRCSum = 0;
                ptrRecvBuf = m_pRecv;
                if(ptrRecvBuf == NULL || m_nRecvCount < m_uRxLen0){
                    m_RxCritSection.Unlock();
                    return FALSE;
                }
                m_uRxBodyCRCSum = CRC16TableFast(ptrRecvBuf, m_uRxLen0);
                m_RxCritSection.Unlock();
            }
            return TRUE;
        }
    }
    else
        return TRUE;
}

BOOL CFrameChannel::CheckRxEnd()
{
    if (m_bFramed)
    {
        UCHAR  uch;
        USHORT uBcc, uRxSum;

        if (m_uRxLen0 != m_uRxLen)           // Check frame length
            return FALSE;

        uRxSum = m_uRxSum;
        *this >> uBcc;
        if (uBcc != uRxSum)
            return FALSE;

        (*GetChannelObject()) >> uch;
        return (uch == ETX);
    }
    else if (m_bAscii)
    {
        UCHAR uch = 0;
        UCHAR uBcc = 0, uRxSum = 0;
        USHORT uRxSumNew = 0;
        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            uRxSum = m_uRxSum;
            *this >> uBcc;
            if (uBcc != uRxSum)
                return FALSE;

            if (m_uRxLen0 != m_uRxLen)           // Check frame length
                return FALSE;
        }
        else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            if (m_uRxLen0 != m_uRxLen){
                return FALSE;
            }

            *this >> uRxSumNew;
            if (uRxSumNew != m_uRxBodyCRCSum)
                return FALSE;
        }

        //*this >> uch;
        (*GetChannelObject()) >> uch;
        return (uch == TCP_ETX);
    }
    else
        return TRUE;
}

CFrameChannel& CFrameChannel::operator << (char ch)
{
    if (m_bFramed || m_bAscii)
    {
        UCHAR uchBuf[2];
        CharToHexStr(ch, uchBuf);
        for (int i = 0; i < 2; i++)
            (*GetChannelObject()) << uchBuf[i];
    }
    else
        (*GetChannelObject()) << ch;

    return *this;
}

CFrameChannel& CFrameChannel::operator >> (char& ch)
{
    if (m_bFramed || m_bAscii)
    {
        UCHAR uchBuf[2], uch;
        for (int i = 0; i < 2; i++)
            (*GetChannelObject()) >> uchBuf[i];

        VERIFY(HexStrToChar(uchBuf, uch));

        ch = uch;
    }
    else
        (*GetChannelObject()) >> ch;

    return *this;
}



#ifdef WINDOWS_PLATFORM_USING
void CFrameChannel::PutString(TCHAR* str, int nLen)
{
    USHORT uLen = _tcslen(str);
    *this << uLen;

    for (USHORT i = 0; i < uLen; i++)
        *this << str[i];
}

void CFrameChannel::PutString(TCHAR* str)
{
    USHORT uLen = _tcslen(str);
    *this << uLen;

    for (USHORT i = 0; i < uLen; i++)
        *this << str[i];
}

int CFrameChannel::GetString(TCHAR* str, int nMaxLen)
{
    USHORT uLen, i;
    *this >> uLen;
    if (uLen >= nMaxLen)
        uLen = nMaxLen-1;

    for (i = 0; i < uLen; i++)
        *this >> str[i];

    return (int)uLen;
}
#elif defined(LINUX_PLATFORM_USING)
void CFrameChannel::PutString(UCHAR* str, int nLen)
{
    USHORT uLen;
    uLen = strlen((const char*)str);
    *this << uLen;

    for (USHORT i = 0; i < uLen; i++)
        *this << str[i];
}

void CFrameChannel::PutString(UCHAR* str)
{
    USHORT uLen;
    uLen = strlen((const char*)str);
    *this << uLen;

    for (USHORT i = 0; i < uLen; i++)
        *this << str[i];
}

int CFrameChannel::GetString(UCHAR* str, int nMaxLen)
{
    int uLen, i;

    //USHORT uLen, i;
    *this >> uLen;
    if (uLen >= nMaxLen)
        uLen = nMaxLen-1;

    for (i = 0; i < uLen; i++)
        *this >> str[i];

    return (int)uLen;
}
#endif

//
// 对接收数据的数据帧头部、消息方向标识、消息长度进行预处理
//
BOOL CFrameChannel::Preprocess()
{
    if (!m_bAscii)
        return TRUE;
    int i = 0;
    UCHAR* p = NULL;
    UCHAR uchBuf[4] = {0}, uchStx = 0 , uchMask = 0,uchVersion = 0, uchHeaderCheckSum = 0;
    UCHAR uchId = 0;
    USHORT uLen = 0;
    USHORT uCalcHeadCRCSum = 0;
    USHORT uRxHeadCRCSum = 0;
    UCHAR* pCalcHeadCRCBuf = NULL;
    BOOL bResult = FALSE;

    union
    {
        USHORT u;
        UCHAR uch[2];
    } DataBuf;

    while (GetRecvCount() >= ASCII_MSG_MIN_LEN)
    {
        m_RxCritSection.Lock();
        p = m_pRecv;

        //for (i = 0; i < 2; i++)
        //	uchBuf[i] = *p++;
        //HexStrToChar(uchBuf, uchStx);  // STR
        uchStx =*p++;

        for (i = 0; i < 2; i++){
            uchBuf[i] = *p++;
        }
        HexStrToChar(uchBuf, uchMask);   // 'T'

        if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            pCalcHeadCRCBuf = p;
            uCalcHeadCRCSum = CRC16TableFast(pCalcHeadCRCBuf, 6);

            for (i = 0; i < 2; i++){
                uchBuf[i] = *p++;
            }
            HexStrToChar(uchBuf, uchId);
        }

        for (i = 0; i < 4; i++){
            uchBuf[i] = *p++;
        }
        HexStrToChar(uchBuf, DataBuf.uch[0]);    // Length
        HexStrToChar(&uchBuf[2], DataBuf.uch[1]);
        uLen = DataBuf.u;

        if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            for (i = 0; i < 4; i++){
                uchBuf[i] = *p++;
            }
            HexStrToChar(uchBuf, DataBuf.uch[0]);
            HexStrToChar(&uchBuf[2], DataBuf.uch[1]);
            uRxHeadCRCSum = DataBuf.u;
        }

        m_RxCritSection.Unlock();

        if(m_ProtoVersion == NET_PROTO_VER_1_0){
            if (uchStx != TCP_STX || uchMask != m_uRemoteMask || uLen > ASCII_MSG_MAX_LEN){
                for (i = 0; i < 1; i++){
                    (*GetChannelObject()) >> uchBuf[i];
                }
            }
            else{
                if (GetRecvCount() >= uLen + 10)  // 必须是一帧完整的数据
                    return TRUE;
                else
                    return FALSE;
            }
        }
        else if (m_ProtoVersion == NET_PROTO_VER_2_0) {
            if (uchStx != TCP_STX || uchMask != m_uRemoteMask || uchId != m_uRemoteID
                    || uLen > ASCII_MSG_MAX_LEN || uRxHeadCRCSum != uCalcHeadCRCSum){
                for (i = 0; i < 1; i++){
                    (*GetChannelObject()) >> uchBuf[i];
                }
            }
            else{
                if (GetRecvCount() >= uLen + 18)  // 必须是一帧完整的数据
                    return TRUE;
                else
                    return FALSE;
            }
        }
    }

    return bResult;
}

void CFrameChannel::SetProtocolFormat(BOOL bFramed, BOOL bAscii)
{
    m_bFramed = bFramed;
    m_bAscii = bAscii;
}

USHORT CFrameChannel::CRC16TableFast( UCHAR *ptr, USHORT len )
{
    USHORT usCRC = 0xFFFF;
    UCHAR ch = 0;

    for (USHORT i = 0; i < len; i++)
    {
        ch = *ptr++;
        usCRC = CRCTalbeABS[(ch ^ usCRC) & 15] ^ (usCRC >> 4);
        usCRC = CRCTalbeABS[((ch >> 4) ^ usCRC) & 15] ^ (usCRC >> 4);
    }

    return usCRC;
}


// dq VISION 验证消息头
BOOL CFrameChannel::GetRxHeaderLANXIN()
{
      UCHAR uch1;
      UCHAR uch2;

      m_uRxSum = 0;
      m_uRxLen = 0;
      (*GetChannelObject()) >> uch1;
      (*GetChannelObject()) >> uch2;
      m_uchRxXor = 0;
      if (!(uch1 == 0xAC && uch2 == 0xED))
          return FALSE;
      else
      {
          (*GetChannelObject()) >> m_uchRxLenLANXIN;
          return TRUE;
      }
}

// dq VISION 验证消息尾
BOOL CFrameChannel::CheckRxEndLANXIN()
{
      UCHAR uchBcc,uchRxSum;

     // std::cout<< "m_uchRxLenLANXIN  "<< m_uchRxLenLANXIN<<"m_uRxLen "<<m_uRxLen<<std::endl;
      if (m_uchRxLenLANXIN != (m_uRxLen-2))           // Check frame length
         return FALSE;
      uchRxSum = m_uchRxXor;
      (*GetChannelObject()) >> uchBcc;
      //if (uchBcc != uchRxSum)
      //    return FALSE;
      //else
          return TRUE;

}

