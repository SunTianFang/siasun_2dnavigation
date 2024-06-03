//                        - SRVCOM.H -
//
//   The interface of class "CTcpServerCom", server version.
//
//   Author: Zhang Lei
//   Date:   2006. 12. 28
//

#ifndef __CTcpServerCom
#define __CTcpServerCom

//#include <afxmt.h>
#include "LstnSock.h"
#include "TcpSock.h"
#include"ZTypes.h"

#define MAX_CLIENTS_COUNT     15

class CListeningSocket;
class CTcpServerChannel;

////////////////////////////////////////////////////////////////////////
//   The interface of class "CTcpServerCom".
class  CTcpServerCom
{
private:
   CHAR             m_uchIpBuf[16];    // Buffer for unicode conversion

public:

    CListeningSocket* m_pListenSocket;   // Pointer to the listening socket

	CCriticalSection  m_CritSection;

	friend class CListeningSocket;

	friend class CTcpServerChannel;

public:
	CTcpServerChannel*   m_pChannels[MAX_CLIENTS_COUNT];  // Server channels
	int               m_nCurChannel;                   // Current channel

protected:
	// Get the client ID from its IP address
	int GetClientID(CHAR* szIpAddr);

	// Get the client ID from its socket address
	int GetClientID(SOCKADDR_IN& Peer);

	// Get the client ID from its channel pointer
	int GetClientID(CTcpServerChannel* pChannel);

   CHAR* GetPeerIP(SOCKADDR_IN& Peer);

public:
	// The constructor
	CTcpServerCom();

	// Destructor
	~CTcpServerCom();

	// Initializations
	BOOL Init(UINT uPort);

	// Create a channel
	virtual int CreateChannel(CHAR* szIpAddr, int nChannel = -1);

	// Select current channel
	BOOL SelectChannel(int nChannel);

	// Test whether the client channel is ready for reading
	BOOL RxReady(int nClientID = -1);

	// Get the pointer to the specified channel object
	CTcpServerChannel* GetChannel(int nClientID) {return m_pChannels[nClientID];}

	// Get the IP address of the specified client
	CHAR* GetIpAddr(int nClientID);

	BOOL AcceptConnect(CTcpSocket* pSocket, SOCKADDR_IN& Peer);

	// Close the channel
	BOOL CloseChannel(int nChannel);

	// Defines the actions to do when the channel is created
	virtual void OnConnectChannel(int nChannel) {}

	// Defines the actions to do when the channel is closed
	virtual void OnCloseChannel(int nChannel) {}

	void DoSend();
};
#endif
