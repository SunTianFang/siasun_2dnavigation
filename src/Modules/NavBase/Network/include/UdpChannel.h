//                        - UDPCHANNEL.H -
//
//   The interface of class "CTcpChannel" (Client version).
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#ifndef __CUdpChannel
#define __CUdpChannel

#include "Afxmt.h"
#include "UdpSock.h"
#include "FrmChan.h"
//#include "ZVirtualSerialPortClient.h"

////////////////////////////////////////////////////////////////////////
//   The interface of class "CTcpChannel".
class DllExport CUdpChannel : public CFrameChannel
{
public:
	CString m_strRemoteIp;
	USHORT  m_uRemotePort;

        HANDLE  m_hRecvCalling;     // Event to indicate the MM timer is calling

public:
	SOCKADDR_IN		m_remoteAddress; 
	int				m_remoteAddrSz; 
	static CCriticalSection m_CritSec;

public:
	CUdpSocket* m_pSocket;            // The data socket

	//ZVirtualSerialPortClient* m_pVirtualSerialPort; // The Virtual serial device interface

public:
	// The constructor
    CUdpChannel() {}
	CUdpChannel(CString strRemoteIp, UINT uRemotePort, BOOL bFramed = FALSE);

    virtual BOOL CreateSocket() {return TRUE;}

	// Detach the socket with the channel
    virtual void DetachSocket() {}

    void SetSocket(CUdpSocket* pSocket);

	// 设置下一次进行通讯的远端IP地址
    void SetRemoteIp(CString strHostIp); /*{m_strRemoteIp = strHostIp;}*/

    void SetRemotePort(unsigned int uRemotePort) {m_uRemotePort = uRemotePort;}

    virtual BOOL InUse();

	// dq VISION
    virtual int Send(UCHAR* pBuf, int nBufLen);
    virtual int Send_Cam(UCHAR* pBuf, int nBufLen);

    virtual int Receive(UCHAR* pBuf, int nBufLen);

};
#endif
