#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H
#include <iostream>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <unistd.h>
#include <vector>
#include "wj_716N_lidar_protocol.h"

using namespace std;
//using boost::asio::ip::tcp;
//using namespace boost::asio;
using namespace wj_lidar;

#define MAX_LENGTH 50000

typedef function<void(unsigned char*, const int)> fundata_t;
class Async_Client
{
public:
    Async_Client(wj_716N_lidar_protocol *protocol);

    Async_Client(fundata_t func);

    ~Async_Client();

    bool connect(string ip, int port);

    bool disconnect();

    void recvData();

    void reconnect();

    bool SendData(unsigned char buf[], int length);

    bool m_bConnected;

    bool m_bReconnecting;

private:
    int m_ifailedRecvCounts;
    wj_716N_lidar_protocol *m_pProtocol;
    string m_sServerIp;
    int m_iServerPort;
    boost::asio::io_service m_io;
    boost::asio::ip::tcp::endpoint m_ep;
    boost::shared_ptr<boost::asio::ip::tcp::socket> m_pSocket;
    boost::system::error_code ec;
    unsigned char m_aucReceiveBuffer[MAX_LENGTH];
    fundata_t m_func;
};

#endif // ASYNC_CLIENT_H
