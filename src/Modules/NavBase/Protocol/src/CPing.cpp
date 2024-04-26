#include "stdafx.h"
#include "CPing.h"

#ifdef WINDOWS_PLATFORM_USING

#include "Ipexport.h"
#include "Icmpapi.h"

CPing::CPing()
{
    WSAData wsaData;
    //Initialize Socket DLL
    if ( WSAStartup( MAKEWORD(1, 1), &wsaData ) != 0) {
        m_bReady = FALSE;
    }
    else
    {
        m_bReady = TRUE;
    }
}

CPing::~CPing()
{
    //Release Socket resource
    WSACleanup();
}

int CPing::Ping(char* strHost)
{
//////////////////////////////////////////////////////////////////////////////////////////////
    if(!m_bReady)
    {
    //	AfxMessageBox( _T("Ping function is not ready!") );
        return -1;
    }
    IPAddr ipAddr = inet_addr( strHost );

    ///////////////////////////////////////////////////////////////////////////////////////
    HANDLE hndlFile;			// Handle for IcmpCreateFile()

    PICMP_ECHO_REPLY pIpe = (PICMP_ECHO_REPLY)GlobalAlloc( GMEM_FIXED | GMEM_ZEROINIT,
                            sizeof(ICMP_ECHO_REPLY));
    if (pIpe == 0)
    {
    //	AfxMessageBox(_T("Failed to allocate PING packet buffer"));
        return -1;
    }
    // Get an ICMP echo request handle
    hndlFile =IcmpCreateFile();
    if ( hndlFile == INVALID_HANDLE_VALUE ) {
    //	AfxMessageBox( _T("Cann't open Ping Service") );
        GlobalFree(pIpe);
        return -1;
    }
    // Reqest an ICMP echo
    DWORD dwStatus = IcmpSendEcho(
        hndlFile,		// Handle from IcmpCreateFile()
        ipAddr,		// Destination IP address
        NULL,			// Pointer to buffer to send
        0,				// Size of buffer in bytes
        NULL,
        pIpe,		// Reply buffer
        sizeof(ICMP_ECHO_REPLY),
        500);			// Time to wait in milliseconds
    //Free allocated memory
    GlobalFree(pIpe);
    //Close PING Service
    IcmpCloseHandle(hndlFile);
    return dwStatus != 0 ? 1 : 0;
}
#endif //WINDOWS_PLATFORM_USING

#ifdef LINUX_PLATFORM_USING
//                     - CPing.cpp -
//
//    ICMP ping . if want to use this ,you must in Root postion .
//    And Then ,  you can make  All Program  in Root postion, and "chmod u+s LinuxAGV"
//
//   Programmer: sfe1012
//   Date:       2016. 12. 10
//
#include "Project.h"
#include "CPing.h"
#include "json/json.h"
#include <fstream>

//#define QT_PINMG

#ifdef QT_PINMG
#include <QProcess>
#endif

CPing::CPing() {

    m_maxPacketSize = 4;
    m_datalen = 56;
    m_nsend = 0;
    m_nreceived = 0;
    m_icmp_seq = 0;

}
CPing::~CPing()
{
}

/*校验和算法*/
unsigned short CPing::getChksum(unsigned short *addr,int len)
{
    int nleft=len;
    int sum=0;
    unsigned short *w=addr;
    unsigned short answer=0;

    /*把ICMP报头二进制数据以2字节为单位累加起来*/
    while(nleft>1)
    {
        sum+=*w++;
        nleft-=2;
    }
    /*若ICMP报头为奇数个字节，会剩下最后一字节。把最后一个字节视为一个2字节数据的高字节，这个2字节数据的低字节为0，继续累加*/
    if( nleft==1)
    {
        *(unsigned char *)(&answer)=*(unsigned char *)w;
        sum+=answer;
    }
    sum=(sum>>16)+(sum&0xffff);
    sum+=(sum>>16);
    answer=~sum;
    return answer;
}

/*设置ICMP报头*/
int CPing::packIcmp(int pack_no, struct icmp* icmp)
{
    int packsize;
    struct icmp *picmp;
    struct timeval *tval;

    picmp = icmp;
    picmp->icmp_type=ICMP_ECHO;
    picmp->icmp_code=0;
    picmp->icmp_cksum=0;
    picmp->icmp_seq=pack_no;
    picmp->icmp_id= m_pid;
    packsize= 8 + m_datalen;
    tval= (struct timeval *)icmp->icmp_data;
    gettimeofday(tval,NULL);    /*记录发送时间*/
    picmp->icmp_cksum=getChksum((unsigned short *)icmp,packsize); /*校验算法*/
    return packsize;
}

/*剥去ICMP报头*/
bool CPing::unpackIcmp(char *buf,int len, struct IcmpEchoReply *icmpEchoReply)
{
    int iphdrlen;
    struct ip *ip;
    struct icmp *icmp;
    struct timeval *tvsend, tvrecv, tvresult;
    double rtt;

    ip = (struct ip *)buf;
    iphdrlen = ip->ip_hl << 2;    /*求ip报头长度,即ip报头的长度标志乘4*/
    icmp = (struct icmp *)(buf + iphdrlen);  /*越过ip报头,指向ICMP报头*/
    len -= iphdrlen;            /*ICMP报头及ICMP数据报的总长度*/
    if(len < 8)                /*小于ICMP报头长度则不合理*/
    {
        printf("ICMP packets\'s length is less than 8\n");
        return false;
    }
    /*确保所接收的是我所发的的ICMP的回应*/
    if( (icmp->icmp_type==ICMP_ECHOREPLY) && (icmp->icmp_id == m_pid) )
    {

        tvsend=(struct timeval *)icmp->icmp_data;
        gettimeofday(&tvrecv,NULL);  /*记录接收时间*/
        tvresult = tvSub(tvrecv, *tvsend);  /*接收和发送的时间差*/
        rtt=tvresult.tv_sec*1000 + tvresult.tv_usec/1000;  /*以毫秒为单位计算rtt*/
        icmpEchoReply->rtt = rtt;
        icmpEchoReply->icmpSeq = icmp->icmp_seq;
        icmpEchoReply->ipTtl = ip->ip_ttl;
        icmpEchoReply->icmpLen = len;
        return true;
    }
    else {
        return false;
    }
}

/*两个timeval结构相减*/
struct timeval CPing::tvSub(struct timeval timeval1,struct timeval timeval2)
{
    struct timeval result;
    result = timeval1;
    if (result.tv_usec < timeval2.tv_usec < 0)
    {
        --result.tv_sec;
        result.tv_usec += 1000000;
    }
    result.tv_sec -= timeval2.tv_sec;
    return result;
}

/*发送三个ICMP报文*/
bool CPing::sendPacket()
{
    size_t packetsize;
    while( m_nsend < m_maxPacketSize)
    {
        m_nsend++;
        m_icmp_seq++;
        packetsize = packIcmp(m_icmp_seq, (struct icmp*)m_sendpacket); /*设置ICMP报头*/

        if(sendto(m_sockfd,m_sendpacket, packetsize, 0, (struct sockaddr *) &m_dest_addr, sizeof(m_dest_addr)) < 0  )
        {
            perror("sendto error");
            continue;
        }
    }
    return true;
}

/*接收所有ICMP报文*/
bool CPing::recvPacket(PingResult &pingResult)
{
    int len;
    extern int errno;
    struct IcmpEchoReply icmpEchoReply;
    int maxfds = m_sockfd + 1;
    int nfd  = 0;
    fd_set rset;
    FD_ZERO(&rset);
    socklen_t fromlen = sizeof(m_from_addr);
    struct timeval timeout;
    timeout.tv_sec = 4;
    timeout.tv_usec = 0;

    for (int recvCount = 0; recvCount < m_maxPacketSize; recvCount++)
    {
        //printf("begin recv\n");
        FD_SET(m_sockfd, &rset);
        if ((nfd = select(maxfds, &rset, NULL, NULL, &timeout)) == -1) {
            perror("select error");
            continue;
        }
        if (nfd == 0) {
            //recv time out
            //printf("request timeout\n");
            icmpEchoReply.isReply = false;
            pingResult.icmpEchoReplys.push_back(icmpEchoReply);
            continue;
        }
        if (FD_ISSET(m_sockfd, &rset)) {
            if( (len = recvfrom(m_sockfd, m_recvpacket,sizeof(m_recvpacket),0, (struct sockaddr *)&m_from_addr,&fromlen)) <0)
            {
                if(errno==EINTR)
                    continue;
                perror("recvfrom error");
                continue;
            }
            icmpEchoReply.fromAddr = inet_ntoa(m_from_addr.sin_addr) ;
            if (icmpEchoReply.fromAddr != pingResult.ip) {
                //printf("invalid address, discard\n");
                //retry again
                recvCount--;
                continue;
            }
        }
        if(unpackIcmp(m_recvpacket, len, &icmpEchoReply)==-1) {
            //retry again
            recvCount--;
            continue;
        }
        icmpEchoReply.isReply = true;
        pingResult.icmpEchoReplys.push_back(icmpEchoReply);
        m_nreceived++;
    }
    return true;
}

bool CPing::getsockaddr(const char * hostOrIp, struct sockaddr_in* sockaddr) {
    struct hostent *host;
    struct sockaddr_in dest_addr;
    unsigned long inaddr=0l;
    bzero(&dest_addr,sizeof(dest_addr));
    dest_addr.sin_family=AF_INET;
    /*判断是主机名还是ip地址*/
    if( inaddr=inet_addr(hostOrIp)==INADDR_NONE)
    {
        if((host=gethostbyname(hostOrIp))==NULL) /*是主机名*/
        {
            //printf("gethostbyname error:%s\n", hostOrIp);
            return false;
        }
        memcpy( (char *)&dest_addr.sin_addr,host->h_addr,host->h_length);
    }
    /*是ip地址*/
    else if (!inet_aton(hostOrIp, &dest_addr.sin_addr)){
        //memcpy( (char *)&dest_addr,(char *)&inaddr,host->h_length);
        //fprintf(stderr, "unknow host:%s\n", hostOrIp);
        return false;
    }
    *sockaddr = dest_addr;
    return true;
}
bool CPing::ping(std::string host, int count, PingResult& pingResult) {

    struct protoent *protocol;
    int size = 50 * 1024;

    m_nsend = 0;
    m_nreceived = 0;
    pingResult.icmpEchoReplys.clear();
    m_maxPacketSize = count;
    m_pid = getpid();

    pingResult.dataLen = m_datalen;

    if( (protocol = getprotobyname("icmp") )==NULL)
    {
        perror("getprotobyname");
        return false;
    }
    /*生成使用ICMP的原始套接字,这种套接字只有root才能生成*/
    if( (m_sockfd=socket(AF_INET,SOCK_RAW,protocol->p_proto) )<0)
    {
        perror("socket error");
        extern int errno;
        pingResult.error = strerror(errno);
        return false;
    }
    /*扩大套接字接收缓冲区到50K这样做主要为了减小接收缓冲区溢出的
      的可能性,若无意中ping一个广播地址或多播地址,将会引来大量应答*/
    setsockopt(m_sockfd,SOL_SOCKET,SO_RCVBUF,&size,sizeof(size) );

    /*获取main的进程id,用于设置ICMP的标志符*/
    if (!getsockaddr(host.c_str(), &m_dest_addr)) {
        pingResult.error = "unknow host " + host;
        return false;
    }
    pingResult.ip = inet_ntoa(m_dest_addr.sin_addr);
    sendPacket();  /*发送所有ICMP报文*/
    recvPacket(pingResult);  /*接收所有ICMP报文*/
    pingResult.nsend = m_nsend;
    pingResult.nreceived = m_nreceived;
    close(m_sockfd);

    return true;

}
void CPing::showPingResult(PingResult pingResult) {
    for (unsigned int icmpEchoReplyIndex = 0; icmpEchoReplyIndex < pingResult.icmpEchoReplys.size(); icmpEchoReplyIndex++) {
        IcmpEchoReply icmpEchoReply = pingResult.icmpEchoReplys.at(icmpEchoReplyIndex);
        if (icmpEchoReply.isReply) {
            printf("%d byte from %s: icmp_seq=%u ttl=%d rtt=%.3f ms\n",
                    icmpEchoReply.icmpLen,
                    icmpEchoReply.fromAddr.c_str(),
                    icmpEchoReply.icmpSeq,
                    icmpEchoReply.ipTtl,
                    icmpEchoReply.rtt);
        } else {
            printf("request timeout\n");
        }
    }
}

int CPing::Ping(std::string strHost)
{
    return 1;

#ifdef QT_PINMG
    QString ip(strHost);
    //Linux指令 "ping -s 1 -c 1 IP"
    QString cmdstr = QString("ping -s 1 -c 1 %1").arg(ip);
    //Windows指令 "ping IP -n 1 -w 超时(ms)"
    //QString cmdstr = QString("ping %1 -n 1 -w %2").arg(ip).arg(1000);
    QProcess cmd;
    cmd.start(cmdstr);
    cmd.waitForReadyRead(1000);
    cmd.waitForFinished(1000);
    QString response = cmd.readAll();
    if (response.indexOf("ttl") == -1)
        return 0;
    else
        return 1;
#else
    string strPingHost(strHost);
    PingResult pingResult;
    bool bRet = ping(strPingHost, 1, pingResult);

    if(!bRet)
    {
       return 0;
    }
    bool bIsOK = false;
    for (unsigned int icmpEchoReplyIndex = 0; icmpEchoReplyIndex < pingResult.icmpEchoReplys.size(); icmpEchoReplyIndex++)
    {
        IcmpEchoReply icmpEchoReply = pingResult.icmpEchoReplys.at(icmpEchoReplyIndex);
        if (icmpEchoReply.isReply)
        {
            bIsOK = true;
        }
    }
    return bIsOK != false ? 1 : 0;
#endif
}
//#include "CPing.h"
//int main(int argc,char *argv[])
//{
//    if (argc != 2) {
//        printf("usage: ping <host or ip>\n");
//        return 0;
//    }
//    char * hostOrIp =  argv[1];
//    int nsend = 0, nreceived = 0;
//    bool ret;
//    PingResult pingResult;
//    Ping ping = Ping();
//    for (int count = 1; count <= 4; count++) {
//        ret = ping.ping(hostOrIp, 1, pingResult);
//        if (count == 1) {
//            printf("PING %s(%s): %d bytes data in ICMP packets.\n",hostOrIp, pingResult.ip.c_str(), pingResult.dataLen);
//        }
//        if (!ret) {
//            printf("%s\n", pingResult.error.c_str());
//            break;
//        }
//        ping.showPingResult(pingResult);
//        nsend += pingResult.nsend;
//        nreceived += pingResult.nreceived;
//    }
//    if (ret) {
//        printf("%d packets transmitted, %d received , %%%d lost\n", nsend, nreceived,
//            (nsend - nreceived) / nsend * 100);
//    }
//    return 0;
//}


void* PingProc(LPVOID pParam)
{
     CPing* oLaserScanner = (CPing*)pParam;
     while(sem_trywait(oLaserScanner->m_hKillThread) != WAIT_OBJECT_0 )
     {
        oLaserScanner->SupportRoutine();
        Sleep(1000);
     }
     SetEvent(oLaserScanner->m_hThreadDead);

   return NULL;
}

#ifdef QT_PINMG
bool CPing::QtPing(const char *strHost , QString &response)
{
    QString ip(strHost);
    //Linux指令 "ping -s 1 -c 1 IP"
    QString cmdstr = QString("ping -s 1 -c 1 %1").arg(ip);
    //Windows指令 "ping IP -n 1 -w 超时(ms)"
    //QString cmdstr = QString("ping %1 -n 1 -w %2").arg(ip).arg(1000);
    QProcess cmd;
    cmd.start(cmdstr);
    cmd.waitForReadyRead(1000);
    cmd.waitForFinished(1000);
    /*QString*/ response = cmd.readAll();
    if (response.indexOf("ttl") == -1)
        return false;
    else
        return true;
}
#endif

bool CPing::InitAllIp()
{
    std::ifstream initFile(WORK_PATH"exParam.json");
    navip = "null";
    agvip = "null";
    ctrlip = "null";
    ctrlip1 = "null";
    saip = "null";
    apip = "null";
    float forwardRevise = 0.0f;
    float backwardRevise = 0.0f;
    if(initFile)
    {
        Json::Reader read;
        Json::Value root;
        read.parse(initFile, root, false);
        if(!root["IP"]["NAV"].isNull())
        {
            navip = root["IP"]["NAV"].asString();
        }
        if(!root["IP"]["AGV"].isNull())
        {
            agvip = root["IP"]["AGV"].asString();
        }
        if(!root["IP"]["CTRL"].isNull())
        {
            ctrlip = root["IP"]["CTRL"].asString();
        }
        if(!root["IP"]["CTRL1"].isNull())
        {
            ctrlip1 = root["IP"]["CTRL1"].asString();
        }
        if(!root["IP"]["SA"].isNull())
        {
            saip = root["IP"]["SA"].asString();
        }
        if(!root["IP"]["AP"].isNull())
        {
            apip = root["IP"]["AP"].asString();
        }
        if(!root["NAVPARAM"]["ForwardRevise"].isNull())
        {
            forwardRevise = root["NAVPARAM"]["ForwardRevise"].asFloat();
        }
        if(!root["NAVPARAM"]["BackwardRevise"].isNull())
        {
            backwardRevise = root["NAVPARAM"]["BackwardRevise"].asFloat();
        }
        initFile.close();
    }
    else {
        return false;
    }

    return true;
}
bool CPing::Install()
{
    if(!InitAllIp())
        return false;
    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);
    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);
    if(pthread_create(&m_PingThread,NULL,PingProc,(void *)this) != 0)
    {
        std::cout<<"Creat PingProc Pthread Failed"<<std::endl;
        return FALSE;
    }
    else
    {
        std::cout<<"Creat PingProc Pthread OK"<<std::endl;
    }
}
void  CPing::Unstall()
{
    if (m_hKillThread != NULL)
    SetEvent(m_hKillThread);

    if(m_hThreadDead)
    while (WaitForSingleObject(m_hThreadDead, 3000) != WAIT_OBJECT_0);

    if (m_hKillThread != NULL)
    CloseHandle(m_hKillThread);
    SAFE_DEL(m_hKillThread);

    if (m_hThreadDead != NULL)
    CloseHandle(m_hThreadDead);
    SAFE_DEL(m_hThreadDead);
}


void CPing::SupportRoutine()
{
#ifdef QT_PINMG
    QString response;
    if(!QtPing(navip.c_str(),response))
    {
    }
    else
    {
    }

    if(!QtPing(ctrlip.c_str(),response))
    {
    }
    else
    {
    }

    if(!QtPing(saip.c_str(),response))
    {
    }
    else
    {
    }

//    if(!QtPing(apip.c_str()))
//    {
//    }
//    else
//    {
//    }
#endif
}

#endif //LINUX_PLATFORM_USING
