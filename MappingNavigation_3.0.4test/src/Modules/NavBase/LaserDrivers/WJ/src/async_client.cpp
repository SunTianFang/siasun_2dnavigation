#include "async_client.h"
#include <boost/exception/all.hpp>

#include "blackboxhelper.hpp"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

Async_Client::Async_Client(wj_716N_lidar_protocol *protocol)
{
    m_ifailedRecvCounts = 0;
    m_pProtocol = protocol;
    m_bConnected = false;
    m_bReconnecting = false;
    m_pSocket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(m_io));
    cout << "TCP-Connection is initialized!" << endl;
}
Async_Client::Async_Client(fundata_t func)
{
    m_func = func;
    m_ifailedRecvCounts = 0;
    m_pProtocol = NULL;
    m_bConnected = false;
    m_bReconnecting = false;
    m_pSocket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(m_io));
    cout << "TCP-Connection is initialized!" << endl;
}
Async_Client::~Async_Client()
{
    disconnect();
}

bool Async_Client::connect(string ip, int port) 
{
//    std::cout << this << "&&&&&&&" << ip << "  " << port << std::endl;
    try
    {
        if (m_bConnected)
        {
            return false;
        }
        if(m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        m_sServerIp = ip;
        m_iServerPort = port;
        m_ep = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port);
        m_pSocket->connect(m_ep,ec);
        if(ec)
        {
            std::cerr << m_pSocket << " " << ec.message() << std::endl;
            m_bConnected = false;
            return false;
        }
        else
        {
            m_bConnected = true;
//            std::cerr << __FILE__ << " " << __LINE__ << " m_bConnected: " << m_bConnected << std::endl;
//            boost::thread recvThread(boost::bind(&Async_Client::recvData,this));
            return true;
        }
    }
    catch (boost::exception &e)
    {
        std::cerr << __FILE__ << " LINE: " << __LINE__ << boost::diagnostic_information(e) << std::endl;
        return false;
    }
}

bool Async_Client::disconnect() 
{
    try
    {
        cout << "Disconnecting connection!" << endl;
        m_bConnected = false;
        if(m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        return true;
    }
    catch(boost::exception& e)
    {
        return false;
    }
}

void Async_Client::recvData()
{
    try
    {
        size_t len = m_pSocket->read_some(boost::asio::buffer(m_aucReceiveBuffer));
        if (len > 0)
        {
            m_ifailedRecvCounts = 0;
//            m_pProtocol->dataProcess(m_aucReceiveBuffer,len);
            m_func(m_aucReceiveBuffer,len);
        }
    }
    catch (boost::exception &e)
    {
        if(m_ifailedRecvCounts++ > 2)
            m_bConnected = false;
        std::cerr << __FILE__ << " LINE: " << __LINE__ << boost::diagnostic_information(e) << std::endl;
    }

}

void Async_Client::reconnect()
{
    m_bReconnecting = true;

    int connect_state = 0;
    if(m_bConnected)
        connect_state = 1;
    else
        connect_state = 0;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "WJ m_bConnected = ",connect_state);
#endif

    while (!m_bConnected)
    {
        if (m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Initializing network! ");
#endif
            cout << "Initializing network!" << endl;
        }
        sleep(3);
        cout << "Start reconnecting laser!" << endl;
        sleep(2);
        if (connect(m_sServerIp, m_iServerPort))
        {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Succesfully connected! ");
#endif
            cout << "Succesfully connected!" << endl;
            break;
        }
        else
        {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Failed to connected! ");
#endif
            cout << "Failed to reconnect!" << endl;
        }
        sleep(2);
    }
    m_bReconnecting = false;
}

bool Async_Client::SendData(unsigned char buf[], int length)
{
    try
    {
        if (m_pSocket->is_open() && m_bConnected)
        {
            size_t st = m_pSocket->send(boost::asio::buffer(buf, length));
            if (st == length)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception e)
    {
        return false;
    }
}
