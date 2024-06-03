#ifndef WJ_716N_LIDAR_PROTOCOL_H
#define WJ_716N_LIDAR_PROTOCOL_H

#include <cstring>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>

using namespace std ;
namespace wj_lidar
{
#define MAX_LENGTH_DATA_PROCESS 200000
typedef struct TagDataCache
{
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
}DataCache;

class wj_716N_lidar_protocol
{
public:
    wj_716N_lidar_protocol();

    bool dataProcess(unsigned char *data,const int reclen);

    bool protocl(unsigned char *data,const int len);

    bool OnRecvProcess(unsigned char *data, int len);

    bool checkXor(unsigned char *recvbuf, int recvlen);

    void send_scan(const char *data,const int len);

    bool setConfig(const int freq);

    bool getFullState() const { return m_bfullscan; }

    unsigned long long getSyncTime() const { return m_synctime; }

    unsigned long long getRawTime() const { return m_rawtime; }

    bool getSyncState() const { return m_bsyncstate; }

    void getData(vector<float> *distance);

    void getIntensity(vector<float> *intensity);

    bool heartstate;
    void analysisTimeStamp(unsigned char *data);

private:
    void movedata(DataCache &sdata);
    DataCache           m_sdata;
    unsigned int        m_u32PreFrameNo;
    unsigned int        m_u32ExpectedPackageNo;
    int                 m_n32currentDataNo;
    float               scandata[1081];
    float               scanintensity[1081];
    int                 total_point;
    int                 freq_scan;
    bool                m_bfullscan;
    unsigned long long  m_synctime;
    unsigned long long  m_rawtime;
    unsigned long long  m_diffvalue;
    bool                m_bsyncstate;
};

}
#endif // WJ_716N_LIDAR_PROTOCOL_H
