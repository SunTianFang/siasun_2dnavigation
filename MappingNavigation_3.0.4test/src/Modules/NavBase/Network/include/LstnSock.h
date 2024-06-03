//                    - LSTNSOCK.H -
//
//   The interface of classes "CListeningSocket".
//
//   Author: Zhang Lei
//   Date:   2006. 10. 31
//

#ifndef __CListeningSocket
#define __CListeningSocket

class CTcpServerCom;
class CTcpServerChannel;

//#if !defined _WIN32_WCE
//#define CCeSocket      CSocket
//#endif
#include "CSocket.h"

#define CCeSocket      CSocket

//#include <afxsock.h>

/////////////////////////////////////////////////////////////////////////////
// The interface of class "CListeningSocket".
class CListeningSocket : public CCeSocket
{
private:
	CTcpServerCom* m_pNetCom;
	
public:
	CListeningSocket(CTcpServerCom* pNetCom);
	virtual ~CListeningSocket();

public:
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CListeningSocket)
	public:
	virtual void OnAccept(int nErrorCode);
	virtual void OnClose(int nErrorCode);

    //add for listen socket
    virtual BOOL Create(UINT uPort);
	//}}AFX_VIRTUAL

	// Generated message map functions
	//{{AFX_MSG(CListeningSocket)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG
};
#endif
