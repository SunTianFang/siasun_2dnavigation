#ifndef DDS_TASK_H
#define DDS_TASK_H


#define TOPIC_HIGH_LASER "/OtherLaser"
#define TOPIC_LOW_LASER  "/Ros/Laser"
#define TOPIC_ROBOT_POSE "/RobotPose"
#define LASER_COUNT 2

#include "Scan.h"
#include "MySubscriber.h"
#include "Localization_Msg.h"
#include "ndt_pointcloud.h"
#include "DDS_DataPublisher.h"
#include "ThreadHelper.h"
#include "StampedPos.h"

class DDSTask : public CThreadHelper
{

private:
    MySubscriber m_DDSSubScriber;
    std::function<void(const NAVPointCloudDDS *const)> m_pDDSCallBack;
    std::mutex Laser_msg_mutex;

    _Localization_Msg LocalizationMsg;
    int16_t*  dis;
    int16_t* lcmintensities;
    unsigned long long cloudLastTime_;
    bool bLocalMsgReady;
//    int nState_LocalMsg = 2;
    int nLasersLineCount;

    int16_t m_iSensorCount;
    bool m_bLaserState[4];
    unsigned long long LaserMsgTick[LASER_COUNT]; //激光时间戳
    DDS_DataPublisher<NAVPointCloudDDSPubSubType> DDSPublisher_high; //高位激光
    DDS_DataPublisher<NAVPointCloudDDSPubSubType> DDSPublisher_low; //低位激光

    DDS_DataPublisher<GeneralDataDDSPubSubType> DDSPublisher_robotPose; //


    float MaxRange[4];
    float MinRange[4];

    sensor::CRawScan scans;

    uint64_t m_robotStamp;

    uint64_t m_laserStamp1;

    uint64_t m_laserStamp2;

    long long int m_stampLastSend;

public:
    DDSTask(){
//        nState_LocalMsg = 0;
//        LocalMsgTick = 0;
        dis = nullptr;
        lcmintensities = nullptr;
        nLasersLineCount = 0;

    }
    ~DDSTask(){
        if(lcmintensities!=nullptr)
        {
            delete[] lcmintensities;
            delete[] dis;
        }
    }
    DDSTask(const DDSTask&)=delete;  //禁止拷贝构造函数
    DDSTask& operator=(const DDSTask&)=delete; // 禁止拷贝赋值运算符
    static DDSTask& GetDDSInstance(){
        static DDSTask instance;
        return instance;
    }


    void DDSInit();
    bool LoadLaserParam();
    void SendMessage(NAVPointCloudDDS PubType, std::string TopicName);
    bool GetLocalizationMsg(sensor::CRawScan& pScan,CStampedPos &robotPosture);
    void DDSSendLocalizationMsg();


protected:
    virtual void SupportRoutineProxy();
};

#endif //DDS_TASK_H
