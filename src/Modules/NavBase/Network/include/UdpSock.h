//                     - UDPSOCK.H -
//
//   The interface of classes "CUdpSocket".
//
//   Author: Zhang Lei
//   Date:   2006. 11. 3
//

#ifndef __CUdpSocket
#define __CUdpSocket

//#include <afxtempl.h>
//#include <afxmt.h>
//#include <afxsock.h>

#include<pthread.h>
#include<semaphore.h>
#include<iostream>
#include"CSocket.h"
#include "ZTypes.h"
#include <list>
using namespace std;

#define CList list

#define MAX_UDP_FRAME_LEN        1024
#define MAX_UDP_LIST_LEN         100

struct CUdpMsg
{
//   CString strSockAddr;
   string  strSockAddr;
   UINT    uPort;
   int     nLen;
   UCHAR   uchMsg[MAX_UDP_FRAME_LEN];
//   CUdpMsg & operator = (const CUdpMsg &Input)
//   {
//	   strSockAddr = Input.strSockAddr;
//	   uPort = Input.uPort;
//	   nLen = Input.nLen;
//	   for(int i = 0  ; i < MAX_UDP_FRAME_LEN; i++ )
//		   uchMsg[i] = Input.uchMsg[i];
//	   return *this;
//   }
};

/////////////////////////////////////////////////////////////////////////////
// The interface of class "CUdpSocket".
class  CUdpSocket : public CSocket
{
	protected:
	//   CList<CUdpMsg, CUdpMsg&> m_MsgList;
	   CList<CUdpMsg> 			m_MsgList;
	   int                      m_nEchoCount;
	//#if defined _WIN32_WCE
	public:
		HANDLE          m_hKillThread;       // Handle of "Kill thread" event
		HANDLE          m_hThreadDead;       // Handle of "Thread dead" event
	//#endif
		   pthread_t m_UdpSocketThread;
	public:
		CUdpSocket();
		virtual ~CUdpSocket();

		BOOL Create(UINT uLocalPort);

//		virtual void OnReceive(int nErrorCode);
		virtual void DoReceive();

		int GetEchoCount() {return m_nEchoCount;}
};
#endif
