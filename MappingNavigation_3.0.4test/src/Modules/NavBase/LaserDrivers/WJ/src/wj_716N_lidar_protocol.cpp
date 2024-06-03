#include "wj_716N_lidar_protocol.h"
#include <iostream>
#include "Tools.h"

#include "blackboxhelper.hpp"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

namespace wj_lidar
{
    bool wj_716N_lidar_protocol::setConfig(const int freq)
    {
//        angle_increment = 0.017453 / 4;
        if (freq == 15) //0.25°_15hz
        {
            freq_scan = 1;
//            time_increment = 1 / 15.00000000 / 1440;
            total_point = 1081;
        }
        else if (freq == 25) //0.25°_25hz
        {
            freq_scan = 2;
//            time_increment = 1 / 25.00000000 / 1440;
            total_point = 1081;
        }

        return true;
    }

    wj_716N_lidar_protocol::wj_716N_lidar_protocol()
    {
        memset(&m_sdata, 0, sizeof(m_sdata));

        m_bfullscan = false;
        freq_scan = 1;
        m_u32PreFrameNo = 0;
//        m_u32ExpectedPackageNofreq_scan = 0;
        m_n32currentDataNo = 0;
        total_point = 1081;
        m_synctime = 0;
        m_rawtime = 0;
        m_diffvalue = GetRealTimeTickCount() - GetTickCount();
        m_bsyncstate = false;
        cout << "wj_716N_lidar_protocl start success" << endl;
    }

    bool wj_716N_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
    {
        if (reclen > MAX_LENGTH_DATA_PROCESS)
        {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
            return false;
        }

        if (m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
        {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
            return false;
        }
        memcpy(&m_sdata.m_acdata[m_sdata.m_u32in], data, reclen * sizeof(char));
        m_sdata.m_u32in += reclen;
        while (m_sdata.m_u32out < m_sdata.m_u32in)
        {
            if (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out + 1] == 0xAA)
            {
                unsigned l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8) |
                                        (m_sdata.m_acdata[m_sdata.m_u32out + 3] << 0);
                l_u32reallen = l_u32reallen + 4;

                if (l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
                {
                    if (OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out], l_u32reallen))
                    {
                        m_sdata.m_u32out += l_u32reallen;
                    }
                    else
                    {
                        cout << "continuous search frame header" << endl;
                        m_sdata.m_u32out++;
                    }
                }
                else if (l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
                {
                    m_sdata.m_u32out++;
                }
                else
                {
                    break;
                }
            }
            else
            {
                m_sdata.m_u32out++;
            }
        } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

        if (m_sdata.m_u32out >= m_sdata.m_u32in)
        {
            m_sdata.m_u32out = 0;
            m_sdata.m_u32in = 0;
        }
        else if (m_sdata.m_u32out < m_sdata.m_u32in && m_sdata.m_u32out != 0)
        {
            movedata(m_sdata);
        }
        return true;
    }

    void wj_716N_lidar_protocol::movedata(DataCache &sdata)
    {
        for (int i = sdata.m_u32out; i < sdata.m_u32in; i++)
        {
            sdata.m_acdata[i - sdata.m_u32out] = sdata.m_acdata[i];
        }
        sdata.m_u32in = sdata.m_u32in - sdata.m_u32out;
        sdata.m_u32out = 0;
    }

    bool wj_716N_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
    {
        if (len > 0)
        {
            if (checkXor(data, len))
            {
                protocl(data, len);
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
        return true;
    }
void wj_716N_lidar_protocol::analysisTimeStamp(unsigned char *data)
{
    uint32_t g_n32TimeStampSec = (unsigned int)((data[14] & 0xff) << 24) +                                //时间戳秒
                (unsigned int)((data[15]& 0xff) << 16) +
                (unsigned int)((data[16]& 0xff) << 8) +
                (unsigned int)(data[17] & 0xff );
    uint32_t g_n32TimeStampSubSec = (unsigned int)((data[18] & 0xff) << 24) +                             //时间戳亚秒
                (unsigned int)((data[19]& 0xff) << 16) +
                (unsigned int)((data[20]& 0xff) << 8) +
                (unsigned int)(data[21] & 0xff );
    if(g_n32TimeStampSec != 0)
    {
       uint64_t res = NTPToUTC(g_n32TimeStampSec, g_n32TimeStampSubSec) ;

       m_synctime = res - m_diffvalue;
       unsigned long int timeStamp = GetTickCount();
       int64_t delta = m_synctime - timeStamp;
       //int64_t delta = timeStamp - m_nSyncTime;
//#ifdef USE_BLACK_BOX
//            FILE_BlackBox(LocBox, "NTPToUTC : ",static_cast<double>(res),"，m_diffvalue ：",static_cast<double>(m_diffvalue),",timeStamp : ",static_cast<double>(timeStamp));
//#endif
       //abs(delta) < 1000
       if(abs(delta) < 2000)
       {
          //  std::cerr << " &&&&&SYNC SUCESS&&&&" << std::endl;
           //abs(g_n32TimeStampSec - SECOND_70_YEAR - GetRealTimeTickCount()/1000) < 2
            m_bsyncstate = true;
            return ;
       }
       else
       {
          // std::cerr << " &&&&&SYNC failed&&&&" << std::endl;
           m_synctime = GetTickCount();
           m_bsyncstate =  false;
           return ;
       }

    }
    else
        m_bsyncstate =  true;
    return ;
}
    bool wj_716N_lidar_protocol::protocl(unsigned char *data, const int len)
    {
        if ((data[22] == 0x02 && data[23] == 0x02) || (data[22] == 0x02 && data[23] == 0x01)) //command type:0x02 0x01/0X02
        {
            heartstate = true;


            unsigned int l_n32TotalPackage = data[80];
            unsigned int l_n32PackageNo = data[81];
            unsigned int l_u32FrameNo = (data[75] << 24) + (data[76] << 16) + (data[77] << 8) + data[78];
            int l_n32PointNum = (data[83] << 8) + data[84];
            int l_n32Frequency = data[79];

            if(l_n32Frequency != freq_scan)
            {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "The scan frequency does not match the one you setted!");
#endif
                cout << "The scan frequency does not match the one you setted!"<< endl;
                return false;
            }

            if(m_u32PreFrameNo != l_u32FrameNo)
            {
                m_u32PreFrameNo = l_u32FrameNo;
                m_u32ExpectedPackageNo = 1;
                m_n32currentDataNo = 0;
				analysisTimeStamp(data);
            }

            if (l_n32PackageNo == m_u32ExpectedPackageNo && m_u32PreFrameNo == l_u32FrameNo)
            {
                if(data[82] == 0x00) //Dist
                {
                    for (int j = 0; j < l_n32PointNum; j++)
                    {
                        scandata[m_n32currentDataNo] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                       ((unsigned char)data[86 + j * 2]);
//                        scandata[m_n32currentDataNo] /= 1000.0;
                        scanintensity[m_n32currentDataNo] = 0;
//                        if (scandata[m_n32currentDataNo] > scan.range_max || scandata[m_n32currentDataNo] < scan.range_min || scandata[m_n32currentDataNo] == 0)
//                        {
//                            scandata[m_n32currentDataNo] = NAN;
//                        }
                        m_n32currentDataNo++;
                    }
                    m_u32ExpectedPackageNo++;
                }
                else if(data[82] == 0x01 && m_n32currentDataNo >= total_point) //intensities
                {
                    for (int j = 0; j < l_n32PointNum; j++)
                    {
                        scanintensity[m_n32currentDataNo - total_point] = (((unsigned char)data[85 + j * 2]) << 8) +
                                                                          ((unsigned char)data[86 + j * 2]); //wt_test_intensity 20230217 WJ716 700

                        if(scanintensity[m_n32currentDataNo - total_point] > 700)
                            scanintensity[m_n32currentDataNo - total_point] = 255;
                        else
                            scanintensity[m_n32currentDataNo - total_point] = 0;

                        m_n32currentDataNo++;
                    }
                    m_u32ExpectedPackageNo++;
                }
                //
//                std::cerr << "m_u32ExpectedPackageNo : " << m_u32ExpectedPackageNo << " l_n32TotalPackage: " << l_n32TotalPackage << std::endl;
                if(m_u32ExpectedPackageNo - 1 == l_n32TotalPackage)
                {
//#ifdef USE_BLACK_BOX
//            FILE_BlackBox(LocBox, "The laser  get fullscandata");
//#endif
                    m_rawtime = GetTickCount();

                    m_bfullscan = true;
//                    m_synctime = NTPToUTC(senconds, picoseconds) - m_diffvalue;

//                    std::cerr << __FUNCTION__ << " line:" << __LINE__ << " " << this << " m_synctime: " << m_synctime << " m_rawtime: " << m_rawtime << std::endl;

                }
                else
                {

                    m_bfullscan = false;

                }
            } 
            return true;
        }
        else
        {
            return false;
        }
    }

    bool wj_716N_lidar_protocol::checkXor(unsigned char *recvbuf, int recvlen)
    {
        int i = 0;
        unsigned char check = 0;
        unsigned char *p = recvbuf;
        int len;
        if (*p == 0xFF)
        {
            p = p + 2;
            len = recvlen - 6;
            for (i = 0; i < len; i++)
            {
                check ^= *p++;
            }
            p++;
            if (check == *p)
            {
                return true;
            }
            else
                return false;
        }
        else
        {
            return false;
        }
    }

    void wj_716N_lidar_protocol::getData(vector<float> *distance)
    {
        for(int i = 0; i < 1081; ++i)
        {
            distance->emplace_back(scandata[i]);
        }
    }

    void wj_716N_lidar_protocol::getIntensity(vector<float> *intensity)
    {
        for(int i = 0; i < 1081; ++i)
        {
            intensity->emplace_back(scanintensity[i]);
        }
    }
}

