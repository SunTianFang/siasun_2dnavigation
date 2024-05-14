#pragma once

#include "Tools.h"
#include "lcm/lcm.h"
#include "ThreadHelper.h"
#include "RawMap.h"
#include "Localization_Msg.h"
//for test
#include "LocalizationTest_Msg.h"
#include "PlatformControl.h"
#include "LinuxSetting.h"
#include "FeatureMatchInfo.h"
#include "FeatureMatchInfoLcm_.h"
#include "LcmCloudAdjusted.h"
#include "ndt_pointcloud.h"
#include "Scan.h"

// for lcm to pad
//#include "LcmToPadType.h"

#include "type.h"
#include "../../NavBase/LaserSensor/include/laser_t.h"

#define LCM_DEFAULT_URL "udpm://239.255.76.67:7667?ttl=2"



bool DeleteArea ( double topLeft_x, double topLeft_y, double downRight_x, double downRight_y );
bool DeleteMultipleAreas (int8_t *iparams, double *dparams);
bool DeleteFasthMapArea(double topLeft_x, double topLeft_y, double downRight_x, double downRight_y);

class LCMTask : public CThreadHelper
{
private:
    _Localization_Msg LocalizationMsg;
    _LocalizationTest_Msg LocalizationMsgTest;
    unsigned long long featureLastTime_;
    unsigned long long cloudLastTime_;
public:
    ~LCMTask(){}
    LCMTask(const LCMTask&)=delete;
    LCMTask& operator=(const LCMTask&)=delete;
    static LCMTask& GetLcmInstance(){
        static LCMTask instance;
        return instance;
    }
    int LCMInit();
    int LCMRecInit();
    int LCMUnInit( void );

    int LCMInitSubscrible(void *user);

    void LCMSendLocalizationMsg();
    bool GetLocalizationMsg(sensor::CRawScan& pScan);
    bool GetLocalizationMsg();
    lcm_t* GetLcmHandle(){
        return lcm;
    }

    bool SendMatchInfo(const CMatchInfo *MatchInfo);
    bool SendFeatureMatchInfo(const CMatchInfo *MatchInfo);

    const CFeatureMatchInfo& GetFeatureMatchInfo();
    void SetFeatureMatchInfo(const FeatureMatchInfoLcm_ *FeatureMatchInfoData);

    bool SendCloudAdjusted(const Eigen::Affine3d initPos,const ndt_oru::CStampedPointCloud cloudIn);
    bool SendGlobalPoindCloud(CPosture robotPosture,const ndt_oru::CStampedPointCloud cloudIn);
    void SetScan(const LcmCloudAdjusted *CloudAdjustData);

    bool SendCalibrationResult(int res,double *lasertf);
    bool WriteLaserParam(void);
    void SendWriteLaserParamResut( bool res );

    CScan * GetCopyScan();

    // lcm 向 pad 发送新的关键帧
    void SendScan ( int id, float* mapinfo, int ptsize, float *ptinfo );
    // lcm 向 pad 发送优化后的地图
    void SendTrajNodePoses(std::map<int,Pose> &optRes);
    void SendSubMapsPoses (int size, vector<Pose> &optRes);
    void SendSubMaps (int submap_index, submap_data pdata, std::vector<double> &blackcell_index, bool bSend);
    // lcm 向 pad 发送应答
    void SendNaviCommand ( int commandId );
    // lcm 向 pad 发送应答 扩展版
    void SendNaviCommandEx ( int commandId, int parm );
    void SendVersion(int commandId);
    void SendStaticObjects ( int commandId, int count, vector<vector<float> > points, vector<CPosture> &psts);
    void SendPlans ( int commandId, int count, vector<vector<float> > plans, vector<int> type);
    // 导航给 pad 发送心跳, 1s发送一次
    void SendHeart();

    bool SendLoadPbResult(int res, vector<int>& submapids);

    bool SendBottomLaserPoindCloud(CPosture robotPosture,const ndt_oru::CStampedPointCloud &cloudIn);

    void SendImageIndex(int index);

    void SendRecImgRes(int mode,int res);

protected:
    virtual void SupportRoutineProxy();

public:
    LCMTask(){
        lcm = lcm_create("udpm://224.0.0.1:7667?ttl=1");
        printf("create ----lcm addr : %p-----\n",lcm);
        nState_LocalMsg = 0;
        LocalMsgTick = 0;
        bLocalMsgReady = false;
        Scan_ = NULL;
        iLocType = -1;
        featureLastTime_ = GetTickCount();
        cloudLastTime_ = GetTickCount();
    }

    int iCalibRes;
    double dLaserPos[12];
    bool  m_bPadReceived;
    lcm_t *lcm;
    int nState_LocalMsg;
    unsigned long long LocalMsgTick;
    bool bLocalMsgReady;
    SIASUN_PTHREAD_T     m_LcmThread;
    SIASUN_PTHREAD_T     m_HeartThread;
    CFeatureMatchInfo featureMatchInfo_;
    std::mutex feature_mutex;
    CScan *Scan_;
    std::mutex Scan_mutex;
    int iLocType;
    vector<int> vecIntensity;
    int16_t* dis;
    int16_t* lcmintensities;

    static bool         m_bNaviToPad;
};

