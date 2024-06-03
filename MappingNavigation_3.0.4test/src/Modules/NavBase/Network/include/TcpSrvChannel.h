//                     - SRVCHANNEL.H -
//
//   The interface of class "CTcpServerChannel".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 31
//

#ifndef __CTcpServerChannel
#define __CTcpServerChannel

class CTcpSocket;
class CTcpServerCom;

#include <afxsock.h>
#include "LstnSock.h"
#include "TcpChannel.h"

/////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTcpServerChannel".
class DllExport CTcpServerChannel : public CTcpChannel
{
public:
	int m_nId;                     // ID number of the client: 0:Host; other: AGV
   CTcpServerCom* m_pServerCom;

public:
	// The constructor
	CTcpServerChannel(CTcpServerCom* pServerCom, int nId, TCHAR* szIpAddr = NULL);

   // Attaches the specified socket to the client
	void AttachSocket(CTcpSocket* pSocket);

	// Detaches the client from its current socket
	virtual void DetachSocket();
};
#endif
