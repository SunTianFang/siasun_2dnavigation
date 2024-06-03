#include "wj_719_lidar_protocol.h"
#include <iostream>
#include <iomanip>
#include "Tools.h"
namespace wj719_lidar
{
    bool wj_719_lidar_protocol::setConfig(const int freq)
    {
        return true;
    }
    wj_719_lidar_protocol::wj_719_lidar_protocol() : \
        g_n32MotorSpeed(0), \
        g_u32PreFrameNo(0), \
        g_n32ScanDatRcvdFrames(0), \
        g_n32ARMSendNo(0)
    {
        g_byteScanFrameBank1 = 0x00; //Bank Num of ScanData
        g_byteScanFrameBank2 = 0x00;
        g_byteScanFrameBank3 = 0x00;
        g_byteScanFrameBank4 = 0x00;
        g_byteScanFrameBank5 = 0x00; //Bank Num of ScanData
        g_byteScanFrameBank6 = 0x00;
        g_byteScanFrameBank7 = 0x00;
        g_byteScanFrameBank8 = 0x00;
        g_byteScanFrameBank9 = 0x00; //Bank Num of ScanData
        g_byteScanFrameBankA = 0x00;
        g_byteScanFrameBankB = 0x00;
        g_byteScanFrameBankC = 0x00;
        memset(&m_sdata,0,sizeof(m_sdata));
        memset(&g_d719scandata,0,sizeof(g_d719scandata));
        memset(&d_n719ScanInden,0,sizeof(g_d719scandata));
        memset(&fScandata719,0,sizeof(fScandata719));

         cout << "wj_719_lidar_protocl start success" << endl;
    }

    bool wj_719_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
    {
         if(reclen > MAX_LENGTH_DATA_PROCESS)
         {
           return false;
         }

         if(m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
         {
           memset(&m_sdata,0,sizeof(m_sdata));
           return false;
         }
         memcpy(&m_sdata.m_acdata[m_sdata.m_u32in],data,reclen*sizeof(char));
         m_sdata.m_u32in += reclen;
         while(m_sdata.m_u32out < m_sdata.m_u32in)
         {
           if(m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0xAA)
           {
             unsigned l_u32reallen = 0;
             if(m_sdata.m_acdata[m_sdata.m_u32out] == 0x02)
             {
                 l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                                (m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                                (m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8)  |
                                (m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
                 l_u32reallen = l_u32reallen + 9;
             }
             else
             {
                 l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8)  |
                                (m_sdata.m_acdata[m_sdata.m_u32out + 3]);
                 l_u32reallen = l_u32reallen + 4;
             }


             if(l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
             {
               if(OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out],l_u32reallen))
               {
                 m_sdata.m_u32out += l_u32reallen;
               }
               else
               {
                 cout << "continuous frame"<<endl;
                 int i;
                 for(i = 1; i<=l_u32reallen; i++)
                 {
                   if((m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0xAA))
                   {
                     m_sdata.m_u32out += i;
                     break;
                   }
                   if(i == l_u32reallen)
                   {
                     m_sdata.m_u32out += l_u32reallen;
                   }
                 }
               }
             }
             else if(l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
             {
               cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS"<<endl;
               cout << "reallen: "<<l_u32reallen<<endl;
               memset(&m_sdata,0,sizeof(m_sdata));

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

         if(m_sdata.m_u32out >= m_sdata.m_u32in)
         {
           memset(&m_sdata,0,sizeof(m_sdata));
         }
          return true;
    }

    bool wj_719_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
    {
         if(len > 0)
         {
           if(checkXor(data,len))
           {
             protocl(data,len);
           }
         }
         else
         {
           return false;
         }
         return true;
    }

    bool wj_719_lidar_protocol::protocl(unsigned char *data, const int len)
    {
       if(data[0] == 0xFF && data[1] == 0xAA)
       {
           if((data[16] == 0x01 && data[17] == 0x04))//ScanData 0.3
           {
              //cout << "Recv ScanData" << endl;
              if (data[2] == 0x04 && data[3] == 0xC5)
              {
                 g_n32ScanDatRcvdFrames++;
                 heartstate = true;
                 //cout << "Analysis ScanData" << endl;
                 int l_n32SerialNo = 0;//Recieved Scandata Number
                 int l_n32ScanCircleNo = 0;
                 l_n32SerialNo = g_n32ScanDatRcvdFrames;
                 g_n32ARMSendNo = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + (data[9]); //Frame Sequence Number
                 int l_n32MoSpeed = ((int)(data[19] << 8) + (int)(data[20]));
                 if (l_n32MoSpeed != 0)
                 {
                     g_n32MotorSpeed = (int)(23437500 / l_n32MoSpeed); //Moto Speed
                 }
                 else
                 {
                     g_n32MotorSpeed = 0;
                 }
                 if (data[18] == 0x01)
                 {
                    g_byteScanFrameBank1 = 0x01;
                    for(int i=0; i<1200; i++)
                    {
                       fScandata719[i] = data[i+21];
                    }
                 }
                 if(g_byteScanFrameBank1)
                 {
                     switch (data[18])
                     {
                        case 0x01:
                           break;
                        case 0x02:
                           g_byteScanFrameBank2 = 0x02;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+1200] = data[i+21];
                           }
                           break;
                        case 0x03:
                           g_byteScanFrameBank3 = 0x03;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+2400] = data[i+21];
                           }
                           break;
                        case 0x04:
                           g_byteScanFrameBank4 = 0x04;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+3600] = data[i+21];
                           }
                           break;
                        case 0x05:
                           g_byteScanFrameBank5 = 0x05;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+4800] = data[i+21];
                           }
                           break;
                        case 0x06:
                           g_byteScanFrameBank6 = 0x06;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+6000] = data[i+21];
                           }
                           break;
                        case 0x07:
                           g_byteScanFrameBank7 = 0x07;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+7200] =  data[i+21];
                           }
                           break;
                        case 0x08:
                           g_byteScanFrameBank8 = 0x08;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+8400] = data[i+21];
                           }
                           break;
                        case 0x09:
                           g_byteScanFrameBank9 = 0x09;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+9600] = data[i+21];
                           }
                           break;
                        case 0x0A:
                           g_byteScanFrameBankA = 0x0A;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+10800] = data[i+21];
                           }
                           break;
                        case 0x0B:
                           g_byteScanFrameBankB = 0x0B;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+12000] = data[i+21];
                           }
                           break;
                        case 0x0C:
                           g_byteScanFrameBankC = 0x0C;
                           for(int i=0; i<1200; i++)
                           {
                              fScandata719[i+13200] = data[i+21];
                           }
                           break;
                        default:
                           break;
                     }
                 }
                 if (g_byteScanFrameBank1 == 0x01
                  && g_byteScanFrameBank2 == 0x02
                  && g_byteScanFrameBank3 == 0x03
                  && g_byteScanFrameBank4 == 0x04
                  && g_byteScanFrameBank5 == 0x05
                  && g_byteScanFrameBank6 == 0x06
                  && g_byteScanFrameBank7 == 0x07
                  && g_byteScanFrameBank8 == 0x08
                  && g_byteScanFrameBank9 == 0x09
                  && g_byteScanFrameBankA == 0x0A
                  && g_byteScanFrameBankB == 0x0B
                  && g_byteScanFrameBankC == 0x0C
                  )
                 {
                     m_rawtime = GetTickCount();
                     m_bfullscan = true;
                    //cout << "Publish Scan Data" << endl;
                    g_byteScanFrameBank1 = 0;
                    g_byteScanFrameBank2 = 0;
                    g_byteScanFrameBank3 = 0;
                    g_byteScanFrameBank4 = 0;
                    g_byteScanFrameBank5 = 0;
                    g_byteScanFrameBank6 = 0;
                    g_byteScanFrameBank7 = 0;
                    g_byteScanFrameBank8 = 0;
                    g_byteScanFrameBank9 = 0;
                    g_byteScanFrameBankA = 0;
                    g_byteScanFrameBankB = 0;
                    g_byteScanFrameBankC = 0;
                    int nDataZhiCount = 0;
                    int nScanDataCount = 0;
                    long nDataJi = 0;
                    float l_nDataY = 0;
                    int nIntensity = 0;
                    double l_64nDataRegionZhiX = 0;
                    double l_64nDataRegionZhiY = 0;

                    for (int m = 0; m < 14400; m = m + 4)
                    {

                        nDataJi = (long)((fScandata719[m + 0] & 0x07) << 17) +
                              (long)((fScandata719[m + 1] & 0xFF) << 9) +
                              (long)((fScandata719[m + 2] & 0xFF) << 1) +
                              (long)((fScandata719[m + 3] & 0x80) >> 7);

                        l_nDataY = (float)(nDataJi/* / 1000.0*/) ;
                        nIntensity = (int)(fScandata719[m + 3] & 0x7F) * 2048;
                        int dataIndex = m/4;
                        g_d719scandata[dataIndex] = l_nDataY;
                        d_n719ScanInden[dataIndex] = nIntensity;

                    }

                 }
                 else
                     m_bfullscan = false;

                 return true;//Analysis ScanData Success
              }
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

    bool wj_719_lidar_protocol::checkXor(unsigned char *recvbuf,  int recvlen)
    {
         int i = 0;
         unsigned char check = 0;
         unsigned char *p = recvbuf;
         int len;

         if(*p == (char)0x02)
         {
               p = p+8;
               len = recvlen - 9;
               for(i = 0; i < len; i++)
               {
                    check ^= *p++;
               }
         }
         else
         {
           p = p+2;
           len = recvlen - 6;
           for(i = 0; i < len; i++)
           {
                check ^= *p++;
           }
           p++;
         }

         if(check == *p)
         {
            return true;
         }
         else
            return false;
        }

    void wj_719_lidar_protocol::getData(vector<float> *distance)
    {
        for(int i = 0; i < 3600; ++i)
        {
            distance->emplace_back(g_d719scandata[i]);
        }
    }

    void wj_719_lidar_protocol::getIntensity(vector<float> *intensity)
    {
        for(int i = 0; i < 3600; ++i)
        {
            intensity->emplace_back(d_n719ScanInden[i]);
        }
    }
}// namespace 719
