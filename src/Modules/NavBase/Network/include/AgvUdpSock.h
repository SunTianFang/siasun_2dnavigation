//                             - CLNTUDPCOM.H -
//
//   The interface of class "CAgvUdpSocket". This class now supports dual traffic controller hot backup.
//
//   Programmed by: Zhanglei
//   Date:          2008. 2. 14
//

#ifndef __CClientUdpCom
#define __CClientUdpCom

#include <Afxmt.h>
#include "UdpSock.h"

class CUdpChannel;
class CUdpServerCom;

class DllExport CAgvUdpSocket : public CUdpSocket
{
private:
    CString m_strHostIp[2];    // The IP address of the stationary computer - Max. no. is 2
    UINT    m_uHostPort;       // Port to communicate with stationary computer
    UINT    m_uMaintainPort;   // Port used to communicate with maintainence program
    DWORD   m_dwLastEchoTime;  // Time of the most recent echo
    int     m_nHostAck;        // ID of the stationary computer from which an echo is received
    //  -1:no echo;  0: Main host;  1: Backup host
    CUdpChannel* m_pChannel;
    CUdpChannel* m_pRLPChannel;

    CUdpServerCom* m_pUdpServerCom;

    UINT    m_uLayoutPort;     // Port used to communicate with layout wizard
    UINT    m_uRLClntPort;
    UINT    m_uRLSrvPort;

    CString m_strRLSrvIp;

public:
    CAgvUdpSocket();

    /*
   BOOL Create(CString strMainHostIp, UINT uHostPort, UINT uMaintainPort, UINT uLocalPort,
                CString strBackupHostIp);
*/
    // 生成Socket对象，并设置通讯端口
    BOOL Create(UINT uLocalPort, UINT uHostPort, UINT uVKIPort);

    // 设置上位机(TC)的IP地址
    void SetHostIp(CString strMainHostIp, CString strBackupHostIp);

    void SetChannel(CUdpChannel* pChannel) {m_pChannel = pChannel;}
    void SetRLPChannel(CUdpChannel* pChannel) {m_pRLPChannel = pChannel;}

    void SetUdpServerCom(CUdpServerCom* pUdpServerCom) {m_pUdpServerCom = pUdpServerCom;}
    virtual void DoReceive();

    void PingHost();
    int GetPingHostAck(DWORD dwTimeOut = 3000);

    // Set the port that be used to communicate with layout wizard
    void SetLayoutPort(UINT uLayoutPort) {m_uLayoutPort = uLayoutPort;}
    void SetRLClntPort(UINT uPort) {m_uRLClntPort = uPort;}
    void SetRLSrvPort(UINT uPort) {m_uRLSrvPort = uPort;}

    void SetRLSrvIp(CString strSrvIp) {m_strRLSrvIp = strSrvIp;}
};

#endif
