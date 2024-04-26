// UdpSock.cpp : implementation file
//

//#include "stdafx.h"
#include "UdpSock.h"
#include"Project.h"
#include"Tools.h"
#include<unistd.h>
#include <poll.h>

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif


void *UdpSupportProc(LPVOID pParam)
{
    CUdpSocket* pSocket = (CUdpSocket*)pParam;
    SOCKET hSocket = pSocket->m_hSocket;
//sfe1012 changes use poll
    struct pollfd fds[1]; //Data structure describing a polling request
    int i;
    /*首先按一定的权限打开两个源文件*/
    fds[0].fd = hSocket;
    /* 指定监听那些文件类型的事件*/
    for (i = 0; i < 1; i++)
    {
        fds[i].events = POLLIN; //There is data to read  指定监听可以读取的文件类型的
    }
    while (WaitForSingleObject(pSocket->m_hKillThread,0) != WAIT_OBJECT_0)
    {

        if (poll(fds, 1, 0) < 0)
        {
            perror("Poll error\n");
        }
        for (i = 0; i< 1; i++)
        {
            if (fds[i].revents) //如果发生变化，可读时 则返回true
            {
                pSocket->DoReceive();
            } /* end of if revents */
        } /* end of for */

        Sleep(10);  // 50
    }
    if(pSocket->m_hThreadDead)
    SetEvent(pSocket->m_hThreadDead);

#ifdef LINUX_PLATFORM_USING
    PthreadExit();
#endif

    return NULL;
}

//#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CUdpSocket".

CUdpSocket::CUdpSocket()
{
	m_UdpSocketThread = 0;
	m_nEchoCount  = 0;
    m_hKillThread = NULL;
     m_hThreadDead = NULL;
}

CUdpSocket::~CUdpSocket()
{
//   m_MsgList.RemoveAll();
	 m_MsgList.clear();

//#if defined _WIN32_WCE
     if(m_hKillThread)
    SetEvent(m_hKillThread);
    while (WaitForSingleObject(m_hThreadDead, 3000) != WAIT_OBJECT_0);

#ifdef LINUX_PLATFORM_USING
    PthreadJoin(m_UdpSocketThread);
#endif

//#endif

	if (m_hKillThread != NULL)
    {
        CloseHandle(m_hKillThread);
        delete m_hKillThread;
        m_hKillThread = NULL;
    }
	if (m_hThreadDead != NULL)
    {
        CloseHandle(m_hThreadDead);
        delete m_hThreadDead;
        m_hThreadDead = NULL;
    }
}

/////////////////////////////////////////////////////////////////////////////
// CUdpSocket member functions

BOOL CUdpSocket::Create(UINT uLocalPort)
{
   m_nEchoCount = 0;
   if (!CSocket::Create(AF_INET, SOCK_DGRAM))
   return FALSE;

   Bind(uLocalPort,AF_INET); // for

    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
   if (m_hKillThread == NULL)
      return FALSE;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
   if (m_hThreadDead == NULL)
      return FALSE;

   /***************************************************************/
    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_UdpSocketThread,&attr,UdpSupportProc,(LPVOID)this) != 0)
    {
        std::cout<<"Creat UdpSupportProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    std::cout<<"BOOL CUdpSocket::Create(UINT uLocalPort): "<<uLocalPort<<std::endl;
    std::cout<<"Creat UdpSupportProc Pthread OK"<<std::endl;
    pthread_attr_destroy(&attr);
  /****************************************************************/

   return TRUE;

}

void CUdpSocket::DoReceive()
{
   CUdpMsg Msg;
   int nLen = ReceiveFrom( Msg.uchMsg, MAX_UDP_FRAME_LEN, Msg.uPort ,Msg.strSockAddr);
   // dq VISION
   //std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ReceiveFrom strSockAddr: "<<Msg.strSockAddr<<" port: "<<Msg.uPort<<std::endl;
   if (nLen != SOCKET_ERROR)
   {
      Msg.nLen = nLen;
      if (m_MsgList.size() >= MAX_UDP_LIST_LEN)
       m_MsgList.pop_front();

      m_MsgList.push_back(Msg);

      m_nEchoCount++;
   }
}
