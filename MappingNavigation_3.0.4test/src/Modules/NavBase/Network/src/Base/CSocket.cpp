/*
 * CSocket.cpp
 *
 *  Created on: 2016-8-10
 *      Author: sfe1012
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include"CSocket.h"
#include"Tools.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>

#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/ip.h>
//#include <linux/tcp.h>
#include <netinet/tcp.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <poll.h>

#include <linux/types.h>
#include <asm/byteorder.h>
//#include <linux/config.h>
//#include <linux/skbuff.h>
#include <linux/ip.h>
//#include <net/sock.h>

#include"Project.h"
#include"Tools.h"

#include<signal.h>
#include <iostream>

#define MAX_BUFFER_SIZE     1024            /* 缓冲区大小*/
#define IN_FILES        1           /* 多路复用输入文件数目*/
#define TIME_DELAY      60          /* 超时时间秒数 */
#define MAX(a, b)       ((a > b)?(a):(b))

//listen the FD
void *SocketThreadProc(void *pParam)
{
    CSocket* pCSocket = (CSocket*)pParam;
    struct pollfd fds[IN_FILES]; //Data structure describing a polling request
    int i;
    /*首先按一定的权限打开两个源文件*/
    fds[0].fd = pCSocket->m_hSocket;
    /* 指定监听那些文件类型的事件*/
    for (i = 0; i < IN_FILES; i++)
    {
        fds[i].events = POLLIN; //There is data to read  指定监听可以读取的文件类型的
    }
    while (WaitForSingleObject(pCSocket->m_hKillThread,0) != WAIT_OBJECT_0)
    {
//        if(fds[0].events)
//        {
//            Specifying a negative value in
//            timeout means an infinite timeout.  Specifying a timeout of zero causes
//            poll() to return immediately, even if no file descriptors are ready.
            if (poll(fds, IN_FILES, 0) < 0)
            {
                perror("Poll error\n");
            }
            for (i = 0; i< IN_FILES; i++)
            {
                if (fds[i].revents) //如果发生变化，可读时 则返回true
                {
                    if(pCSocket->SocketConnected())
                    pCSocket->OnReceive();
                } /* end of if revents */
            } /* end of for */
            Sleep(10);
    }

    if(pCSocket->m_hThreadDead)
    SetEvent(pCSocket->m_hThreadDead);

#ifdef LINUX_PLATFORM_USING
    PthreadExit();
#endif

    std::cout<<"Stop SocketThreadProc OK"<<std::endl;

    return NULL;
}

void CSocket::Start()
{
    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);
    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);

    /***************************************************************/
     pthread_attr_t attr;
     SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
     if(pthread_create(&m_SocketThread,&attr,SocketThreadProc,(void *)this) != 0)
     {
         std::cout<<"Creat SocketThreadProc Pthread Failed"<<std::endl;
         return ;
     }
     std::cout<<"Creat SocketThreadProc Pthread OK"<<std::endl;
     pthread_attr_destroy(&attr);
   /****************************************************************/

}

void CSocket::Stop()
{
    if(m_hKillThread)
    SetEvent(m_hKillThread);

//    if(m_hThreadDead)
//    WaitForSingleObject(m_hThreadDead,9000);

    if(m_hThreadDead)
    while (WaitForSingleObject(m_hThreadDead, 3000) != WAIT_OBJECT_0);

#ifdef LINUX_PLATFORM_USING
    PthreadJoin(m_SocketThread);
#endif

    //std::cout<<" CSocket WaitForSingleObject(m_hThreadDead,9000); OK"<<std::endl;

    if(m_hKillThread)
    CloseHandle(m_hKillThread);

    if(m_hThreadDead)
    CloseHandle(m_hThreadDead);

    if(m_hKillThread != NULL)
    {
        delete m_hKillThread;
        m_hKillThread = NULL;
    }
    if(m_hThreadDead != NULL)
    {
        delete m_hThreadDead;
        m_hThreadDead = NULL;
    }
}

void CSocket::Detach()
{
    Stop(); // add for stop listen receive thread
    Close();
}
CSocket::CSocket()
{
    m_ulConnectTimeOut = 1000; //ms

    m_hSocket = -1;

    m_stHostent  = NULL;

    m_hKillThread = NULL;       // Handle of "Kill thread" event
    m_hThreadDead = NULL;

    m_SocketThread = 0;
}
CSocket:: ~CSocket()
{
    Detach();
}
BOOL CSocket::Create(const int iFamily,const int nSocketType,const int iProtoco,const int iCacheRegionSize)
{
	//Create Socket
//    if ((m_hSocket = socket(iFamily,nSocketType|SOCK_NONBLOCK,iProtoco)) == -1)
//    {
//        perror("socket");
//        return FALSE;
//    }

    if ((m_hSocket = socket(iFamily,nSocketType,iProtoco)) == -1)
    {
        perror("socket");
        return FALSE;
    }
    fcntl(m_hSocket,F_SETFL,fcntl(m_hSocket,F_GETFL,0) | O_NONBLOCK);

//    BOOL bSet=TRUE;
//    setsockopt(m_hSocket,SOL_SOCKET,SO_KEEPALIVE,(const char*)&bSet,sizeof(BOOL));

//    int  set = 1;
//    setsockopt(m_hSocket, SOL_SOCKET, MSG_NOSIGNAL, (void  *)&set, sizeof(int));

    int nRecvBufLen = iCacheRegionSize;
    setsockopt(m_hSocket,SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBufLen, sizeof(int));
    int nSendBufLen = iCacheRegionSize;
    setsockopt(m_hSocket, SOL_SOCKET, SO_SNDBUF, ( const char* )&nSendBufLen, sizeof(int));

//    const char chOpt=1;
//    int   nErr=setsockopt(   m_hSocket,   IPPROTO_TCP,   TCP_NODELAY,   &chOpt,   sizeof(char));

    return TRUE;
}
BOOL CSocket::SetSendBufferZoneSize(const int &iSize)
{
	if(setsockopt( m_hSocket, SOL_SOCKET, SO_SNDBUF, &iSize, sizeof(int)) == -1)
	return FALSE;

	return TRUE;
}
BOOL  CSocket::GetSendBufferZoneSize(int &iSize)
{

	unsigned int optlen = sizeof(iSize);
	if(getsockopt( m_hSocket, SOL_SOCKET, SO_SNDBUF, &iSize, &optlen) == -1)
	return FALSE;

	return TRUE;
}
BOOL CSocket::SetRecBufferZoneSize(const int &iSize)
{
	if(setsockopt( m_hSocket, SOL_SOCKET, SO_RCVBUF, &iSize, sizeof(int)) == -1)
	 return FALSE;

	return TRUE;
}
BOOL  CSocket::GetRecBufferZoneSize(int &iSize)
{
	unsigned int optlen = sizeof(iSize);
	if(getsockopt( m_hSocket, SOL_SOCKET, SO_RCVBUF, &iSize, &optlen) == -1)
	return FALSE;

	return TRUE;
}
BOOL CSocket::Bind( const int nSocketPort , const int iFamily)
{
    /*set sockaddr_in struct Parameter */
    m_ServerSockaddr_in.sin_family = iFamily;
    m_ServerSockaddr_in.sin_port = htons(nSocketPort);
    m_ServerSockaddr_in.sin_addr.s_addr = INADDR_ANY;
    bzero(&(m_ServerSockaddr_in.sin_zero), 8);

    /*使得重复使用本地地址与套接字进行绑定 设置调用closesocket()后,仍可继续重用该socket*/
    int  iReused = 1;
    setsockopt(m_hSocket, SOL_SOCKET, SO_REUSEADDR,&iReused,sizeof(int));

    /*绑定函数bind*/
    if (bind(m_hSocket, (struct sockaddr *)&m_ServerSockaddr_in, sizeof(struct sockaddr))== -1)
    {
        perror("bind");
        return FALSE;
    }
	return TRUE;

}
//
BOOL CSocket::Listen( int nConnectionBacklog )
{
    /*调用listen函数*/
    if (listen( m_hSocket, nConnectionBacklog ) == -1)
    {
        perror("Listen");
        return FALSE;
    }
    return TRUE;
}
BOOL CSocket::Accept(CSocket& rConnectedSocket,sockaddr* lpSockAddr, int* lpSockAddrLen )
{
    /*调用accept函数，等待客户端的连接*/
    if ((m_hSocket = accept(m_hSocket, (struct sockaddr *)&m_AcceptClientSockaddr_in,
                                     (socklen_t*)lpSockAddrLen)) == -1)
    {
        perror("accept");
        return FALSE;
    }
    else
    {
    	lpSockAddr = (struct sockaddr *)&m_AcceptClientSockaddr_in;
    	rConnectedSocket = *this;
    }

	return TRUE;
}
void CSocket::Close()
{
	close(m_hSocket);
}
BOOL CSocket::Connect(const LPCTSTR lpszHostAddress, UINT nHostPort)
{
	/*地址解析函数*/
	if ((m_stHostent = gethostbyname(lpszHostAddress)) == NULL)
	{
		perror("gethostbyname");
		return FALSE;
	}

    /*设置sockaddr_in 结构体中相关参数*/
    m_stServerAddr_in.sin_family = AF_INET;
    m_stServerAddr_in.sin_port = htons(nHostPort);
    m_stServerAddr_in.sin_addr = *((struct in_addr *)m_stHostent->h_addr);
    bzero(&(m_stServerAddr_in.sin_zero), 8);

    //Connect
    /*调用connect函数主动发起对服务器端的连接*/
    unsigned long ulStartTime = GetTickCount();
    bool bIsConnected = false;
    while(!bIsConnected)
    {
         if(connect(m_hSocket,(struct sockaddr *)&m_stServerAddr_in, sizeof(struct sockaddr))== -1 )
         {
             bIsConnected = false;
         }
         else
         {
//             std::cout<<"bIsConnected"<<std::endl;
             bIsConnected = true;
         }
         unsigned long ulEndTime = GetTickCount();
         if((ulEndTime - ulStartTime) > m_ulConnectTimeOut)
         {
        	 return FALSE;
         }
    }

	return TRUE;
}
BOOL CSocket::ShutDown(int nHow)
{
	if(shutdown(m_hSocket,nHow) == -1)
	return FALSE;

	return TRUE;
}
int CSocket::Receive(void* lpBuf, int nBufLen, int nFlags )
{
    memset(lpBuf , 0, nBufLen);
    return recv(m_hSocket,lpBuf,nBufLen,nFlags);  //it is block
}

int CSocket::Send(const void* lpBuf, int nBufLen, int nFlags )
{
//     return  send(m_hSocket,lpBuf , nBufLen , nFlags);
//     for(int i = 0; i < nBufLen; i++)
//         std::cout << (int)((char*)lpBuf)[i] << "-";
//     std::cout << std::endl;
     int iRet = -1;
     if(SocketConnected())
        iRet = send(m_hSocket,lpBuf , nBufLen , nFlags);

     return iRet ;
}

/******************************UDP****************************************/
int CSocket::ReceiveFrom(void* lpBuf, int nBufLen,UINT& rSocketPort, string &rSocketAddress,  int nFlags )
{
    struct sockaddr_in sockaddr_inTEmp;
    unsigned int itolen = sizeof(struct sockaddr);
    int iRet = recvfrom(m_hSocket,lpBuf,nBufLen,nFlags,(struct sockaddr *)&sockaddr_inTEmp,&itolen);
    rSocketPort = ntohs(sockaddr_inTEmp.sin_port);
    rSocketAddress = inet_ntoa(sockaddr_inTEmp.sin_addr);
    if(iRet > 0)
        return iRet;
    else
        return -1;
}

int CSocket::ReceiveFrom(void *lpBuf, int nBufLen, string &rSocketAddress, UINT &rSocketPort, int nFlags)
{
    ReceiveFrom(lpBuf,  nBufLen,rSocketPort, rSocketAddress, nFlags );
}

int CSocket::SendTo(const void* lpBuf, int nBufLen,const USHORT nHostPort,const char * lpszHostAddress , int nFlags)
{
	/*地址解析函数*/
	hostent *stHostentTemp;
	if ((stHostentTemp = gethostbyname(lpszHostAddress)) == NULL)
	{
		perror("gethostbyname");
		return FALSE;
	}
    /*设置sockaddr_in 结构体中相关参数*/
	struct sockaddr_in sockaddr_inTEmp;
	sockaddr_inTEmp.sin_family = AF_INET;
	sockaddr_inTEmp.sin_port = htons(nHostPort);
	sockaddr_inTEmp.sin_addr = *((struct in_addr *)stHostentTemp->h_addr);
    bzero(&(sockaddr_inTEmp.sin_zero), 8);

	return sendto(m_hSocket,lpBuf,nBufLen,nFlags,(struct sockaddr *)&sockaddr_inTEmp,sizeof(struct sockaddr));
}


int CSocket::SocketConnected()
{
    int sock = m_hSocket;
    if(sock<=0)
    return 0;
    struct tcp_info info;
    int len=sizeof(info);
    getsockopt(sock, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len);
    if((info.tcpi_state==TCP_ESTABLISHED))
    {
        //myprintf("socket connected\n");
        return 1;
    }
    else
    {
        //myprintf("socket disconnected\n");
        return 0;
    }
}
