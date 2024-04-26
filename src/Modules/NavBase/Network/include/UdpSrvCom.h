//                        - SRVCOM.H -
//
//   The interface of class "CUdpServerCom", server version.
//
//

#ifndef __CUdpServerCom
#define __CUdpServerCom

#include <Afxmt.h>
#include "ZTypes.h"
#include "UdpSock.h"

#define MAX_UDP_CLIENTS_COUNT     15

class CUdpChannel;

////////////////////////////////////////////////////////////////////////
//   The interface of class "CUdpServerCom".
class DllExport CUdpServerCom
{
private:
    TCHAR               m_uchIpBuf[16];    // Buffer for unicode conversion

public:
    CCriticalSection    m_CritSection;
    HANDLE              m_hRecvCalling;     // Event to indicate the MM timer is calling

    //friend class CUdpChannel;

public:
    CUdpChannel*    m_pChannels[MAX_UDP_CLIENTS_COUNT];  // Server channels
    int             m_nCurChannel;                   // Current channel

    BOOL            m_bConnected;

    BOOL   m_bFramed;
    BOOL   m_bAscii;
    USHORT  m_RemotePort;
    CUdpSocket* m_pUdpSocket;            // The data socket

    char m_chIpStr[32];

    UCHAR  m_chTxId;
    UCHAR  m_chRxId;
    UCHAR  m_chTxMask;
    UCHAR  m_chRxMask;

protected:
    // Get the client ID from its IP address
    int GetClientID(TCHAR* szIpAddr);

    // Get the client ID from its socket address
    int GetClientID(SOCKADDR_IN& Peer);

    // Get the client ID from its channel pointer
    int GetClientID(CUdpChannel* pChannel);

    // Get the client ID from its channel pointer
    int GetClientID(CString strRemoteIp);

    TCHAR* GetPeerIP(SOCKADDR_IN& Peer);

public:
    // The constructor
    CUdpServerCom(USHORT uRemotePort, BOOL bFramed = TRUE, BOOL bAscii = FALSE);

    // Destructor
    ~CUdpServerCom();

    // Initializations
    BOOL Init(UINT uPort);

    void SetUdpSocket(CUdpSocket* pSocket) {m_pUdpSocket = pSocket;}

    void SetProtoID(UCHAR chTxId, UCHAR chRxId)
    {
        m_chTxId = chTxId;
        m_chRxId = chRxId;
    }

    void SetProtoMask(UCHAR chTxMask, UCHAR chRxMask)
    {
        m_chTxMask = chTxMask;
        m_chRxMask = chRxMask;
    }

    // Create a channel
    virtual int CreateChannel(TCHAR* szIpAddr, UINT uPort, int nChannel = -1);

    // Select current channel
    BOOL SelectChannel(int nChannel);

    // Test whether the client channel is ready for reading
    BOOL RxReady(int nClientID = -1);

    // Get the pointer to the specified channel object
    CUdpChannel* GetChannel(int nClientID) {return m_pChannels[nClientID];}

    // Get the IP address of the specified client
    char* GetIpAddr(int nChannel);

    BOOL CreateCom(TCHAR* szIpAddr);

    // Close the channel
    BOOL CloseChannel(int nChannel);

    // Defines the actions to do when the channel is created
    virtual void OnConnectChannel(int nChannel) {}

    // Defines the actions to do when the channel is closed
    virtual void OnCloseChannel(int nChannel) {}

    void DoSend();

    int GetRecvCount(int nClientID = -1);

    BOOL RecvFromBuf(CString strRemoteIp, UCHAR* pBuf, int nLen);

};
#endif
