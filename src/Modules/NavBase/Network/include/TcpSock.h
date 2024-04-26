//                         - TCPSOCK.H -
//
//   The interface of classes "CTcpSocket".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 30
//

#ifndef __CTcpSocket
#define __CTcpSocket

#include <afxsock.h> 
#include "ZTypes.h"
#include "TimeOutSock.h"

class CTcpChannel;

//#if !defined _WIN32_WCE
//#define CCeSocket      CSocket
//#endif
//
//#define CCeSocket      CSocket 

/////////////////////////////////////////////////////////////////////////////
// The interface of class "CTcpSocket".
class DllExport CTcpSocket : public CTimeOutSocket
{
public:
	CTcpChannel* m_pChannel;

public:
	CTcpSocket(CTcpChannel* pChannel = NULL);
   virtual ~CTcpSocket() 
   {
	   m_pChannel = NULL;
   }

   void SetChannel(CTcpChannel* pChannel);

public:
	virtual void OnReceive(int nErrorCode);
	virtual void OnClose(int nErrorCode);
};
#endif
