//                        - TCPCHANNEL.H -
//
//   The interface of class "CTcpChannel" (Client version).
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#ifndef __CTcpChannel
#define __CTcpChannel

#include "Afxmt.h"
#include "TcpSock.h"
#include "FrmChan.h"

////////////////////////////////////////////////////////////////////////
//   The interface of class "CTcpChannel".
class DllExport CTcpChannel : public CFrameChannel
{
public:
	CTcpSocket* m_pSocket;            // The data socket
	SHORT m_bConnected;         // Connected to host or not?
	CCriticalSection m_TcpCritSection;

public:
	// The constructor
	CTcpChannel(BOOL bFramed = FALSE, BOOL bAscii = FALSE);
	~CTcpChannel();

	virtual BOOL CreateSocket();

	// Detach the socket with the channel
   virtual void DetachSocket();

   virtual BOOL InUse() {return (m_pSocket != NULL);}

	// dq VISION
	virtual int Send(UCHAR* pBuf, int nBufLen);
    virtual int Send_Cam(UCHAR* pBuf, int nBufLen);

	virtual int Receive(UCHAR* pBuf, int nBufLen);

	virtual void Disconnect();
};
#endif
