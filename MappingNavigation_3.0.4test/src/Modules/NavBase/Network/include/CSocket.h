/*
 * CSocket.h
 *
 *  Created on: 2016-8-10
 *      Author: sfe1012
 */
#ifndef CSOCKET_H_
#define CSOCKET_H_
#include<ZTypes.h>
#include <netdb.h>
#include<iostream>
#include"NetPort.h"
using std::string;
class CSocket
{
    public:
                 CSocket();
         virtual ~CSocket();
    public:
            // iFamily:AF_INET:IPv4 AF_INET6:IPV6   nSocketType:SOCK_STREAM:TCP  SOCK_DGRAM:UDP  iProtoco : default = 0  OK
            BOOL Create(const int iFamily = AF_INET, const int nSocketType = SOCK_STREAM,
                        const int iProtoco = 0,const int iCacheRegionSize = 1024*1024);
            //Set Send Buffer Size EX:iSize = 10 * 1024  define:10Kb
            BOOL SetSendBufferZoneSize(const int &iSize);

            BOOL GetSendBufferZoneSize(int &iSize);

            BOOL SetRecBufferZoneSize(const int &iSize);

            BOOL GetRecBufferZoneSize(int &iSize);
            //Accept Connect
            virtual BOOL Accept(CSocket& rConnectedSocket,sockaddr* lpSockAddr = NULL, int* lpSockAddrLen = 0);

            BOOL Bind(const int nSocketPort , const int iFamily);

            virtual void Close();

            //input ms
            void SetConnectTimeOutValue(const unsigned long nTime){ m_ulConnectTimeOut = nTime;}
            //Connect
            BOOL Connect(const LPCTSTR lpszHostAddress, UINT nHostPort);
            //nConnectionBacklog the max can connect counts
            BOOL Listen(int nConnectionBacklog=5);

    //		SHUT_RD   = No more receptions;
    //		SHUT_WR   = No more transmissions;
    //		SHUT_RDWR = No more receptions or transmissions.
            BOOL ShutDown(int nHow = SHUT_WR);

            virtual int Receive(void* lpBuf, int nBufLen, int nFlags = 0);

            int ReceiveFrom(void* lpBuf, int nBufLen,UINT& rSocketPort, string & rSocketAddress ,  int nFlags = 0);
            int ReceiveFrom(void* lpBuf, int nBufLen,string &rSocketAddress,UINT& rSocketPort,   int nFlags );

            virtual int Send(const void* lpBuf, int nBufLen, int nFlags = 0);

            int SendTo(const void* lpBuf, int nBufLen,const USHORT  nHostPort, const char * lpszHostAddress = NULL, int nFlags = 0);

            virtual void OnReceive(int nErrorCode = 0){}
            virtual void OnSend(int nErrorCode = 0){}

    //    	virtual void OnOutOfBandData(int nErrorCode);
    //    	virtual void OnAccept(int nErrorCode);
    //    	virtual void OnConnect(int nErrorCode);
    //    	virtual void OnClose(int nErrorCode);

            virtual void Start();
            virtual void Stop();
            virtual void Detach();
            int     SocketConnected();

    public:
            //together use
            int								     m_hSocket;
    private:

            //for server
            struct sockaddr_in                  m_ServerSockaddr_in;
            struct sockaddr_in			        m_AcceptClientSockaddr_in;

            //for Client
            struct hostent 	    	  			*m_stHostent;
            struct sockaddr_in 		   			 m_stServerAddr_in;
            unsigned long					     m_ulConnectTimeOut;

    public:
        HANDLE m_hKillThread;       // Handle of "Kill thread" event
        HANDLE m_hThreadDead;       // Handle of "Thread dead" event
        pthread_t m_SocketThread;
//        pthread_attr_t m_pthread_attr;
};

#endif /* CCESOCKET_H_ */
