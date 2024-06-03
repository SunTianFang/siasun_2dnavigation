//                        - NETCHANNEL.H -
//
//   The interface of class "CClientChannel".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#ifndef __CClientChannel
#define __CClientChannel

#include "TcpChannel.h"

////////////////////////////////////////////////////////////////////////
//   The interface of class "CClientChannel".
class DllExport CClientChannel : public CTcpChannel
{
public:
   UINT  m_uLocalPort;         // Local port number
	int   m_nRemotePort;        // Remote port

public:
	// The constructor
	CClientChannel(BOOL bFramed, BOOL bAscii = FALSE);

	// Destructor
	~CClientChannel();
   
   // Create the object
	virtual BOOL CreateSocket();
	
	// Detach the socket from the channel
	virtual void DetachSocket();

	// Set local ports
	void SetLocalPort(UINT uPort = 0);

   // Set Host computer port
	void SetHostPort(int nPort);

	// Set the host computer IP address
        void SetHostIpAddr(const TCHAR* szAddr);

	// Connect the client with a server
	BOOL Connect();

	// Test whether the client is connected to a server
	BOOL Connected() {return m_bConnected;}

	virtual void Disconnect();
};
#endif
