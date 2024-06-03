//
//   The interface of class "CLocalizeFactory".
//

#pragma once

#include <stdio.h>
#include <mutex>
#include <atomic>
#include "MagicSingleton.h"
#include "Geometry.h"
#include "ZTypes.h"
#include "LCMTask.h"
#include "LocalizationPlan.h"
#include "LocalizationMethods.h"
#include "LocalizationManager.h"
#include "NdtMethod.h"
#include "FeatureMethod.h"
#include "NdtMethod.h"
#include "TemplateMethod.h"
#include"StampedPos.h"
#include "Diagnosis.h"

#include "RecTopvisionImage.h"
#include <json/json.h> 


namespace robo {


enum {
    ONLY_LASER_LOCALIZATION,
    ONLY_TOPVISION_LOCALIZATION,
    LASER_WITH_TOPVISION_LOCALIZATION
};

struct RegionType
{
public:
        RegionType()
        {}
        RegionType(int x0, int y0, int xe, int ye, int type, int uN):
                mStartPtx(x0), mStartPty(y0), mEndPtx(xe), mEndPty(ye), regionType(type), regionUN(uN)
        {}
public:
        float mStartPtx, mStartPty;
        float mEndPtx, mEndPty;
        int regionType;
        int regionUN;
};

class CEvaluate
{
public:
    int min_quality_level;
    unsigned int min_match_num;
    unsigned int min_feature_match_num;
    double good_keep_dist;
    double error_tolerate_dist;
    double fail_tolerate_dist;

public:
    CEvaluate()
    {
        Initialize();
    }

    ~CEvaluate()
    {
    }

    CEvaluate(const CEvaluate& other)
    {
        this->min_quality_level = other.min_quality_level;
        this->min_match_num = other.min_match_num;
        this->good_keep_dist = other.good_keep_dist;
        this->error_tolerate_dist = other.error_tolerate_dist;
        this->fail_tolerate_dist = other.fail_tolerate_dist;
    }

    CEvaluate& operator = (const CEvaluate& Obj)
    {
        this->min_quality_level = Obj.min_quality_level;
        this->min_match_num = Obj.min_match_num;
        this->good_keep_dist = Obj.good_keep_dist;
        this->error_tolerate_dist = Obj.error_tolerate_dist;
        this->fail_tolerate_dist = Obj.fail_tolerate_dist;
        return *this;
    }

    void Initialize()
    {
        min_quality_level = 0;
        min_match_num = 0;
        good_keep_dist = 0.0;
        error_tolerate_dist = 0.0;
        fail_tolerate_dist = 0.0;
    }
};

//For Diagnosis.Add By yu.
class CDiagnosis
{
public:
    bool use_core_dump;
    bool auto_save_log;
    bool publish_point_cloud;
    bool publish_ndt_cell;
    bool publish_reflector;

public:
    CDiagnosis()
    {
        Initialize();
    }

    ~CDiagnosis()
    {
    }

    CDiagnosis(const CDiagnosis& other)
    {
        this->use_core_dump = other.use_core_dump;
        this->auto_save_log = other.auto_save_log;
        this->publish_point_cloud = other.publish_point_cloud;
        this->publish_ndt_cell = other.publish_ndt_cell;
        this->publish_reflector = other.publish_reflector;
    }

    CDiagnosis& operator = (const CDiagnosis& Obj)
    {
        this->use_core_dump = Obj.use_core_dump;
        this->auto_save_log = Obj.auto_save_log;
        this->publish_point_cloud = Obj.publish_point_cloud;
        this->publish_ndt_cell = Obj.publish_ndt_cell;
        this->publish_reflector = Obj.publish_reflector;
        return *this;
    }

    void Initialize()
    {
        use_core_dump = false;
        auto_save_log = false;
        publish_point_cloud = false;
        publish_ndt_cell = false;
        publish_reflector = false;
    }
};


class CFilterParm
{
public:
    bool bEnable;
    std::vector<double> vFactor;

public:
    CFilterParm()
    {
        Initialize();
    }

    ~CFilterParm()
    {
    }

    CFilterParm(const CFilterParm& other)
    {
        this->vFactor.clear();
        this->bEnable = other.bEnable;
        this->vFactor = other.vFactor;
    }

    CFilterParm& operator = (const CFilterParm& Obj)
    {
        this->vFactor.clear();
        this->bEnable = Obj.bEnable;
        this->vFactor = Obj.vFactor;
        return *this;
    }

    void Initialize()
    {
        bEnable = false;
        vFactor.clear();
    }
};

class CLocalizeFactory: public CLocalizationManager
{
private:
    std::mutex  ndt_mtx;
    std::mutex  diagnosis_mtx;
    bool         m_bLocStarted;
    unsigned int localize_method;   //bit0:ndt, bit1:feature, bit2:dataset
    bool         m_bLocFirstTime;
    unsigned int m_nCurSlamStep;
    CEvaluate    m_Evaluate;
    CDiagnosis   m_Diagnosis;
    char version_[8];
    char date_[6];
    CStampedPos    m_PosNowFiltered;             // 当前经过滤波姿态
    CStampedPos    m_PosNowFiltered_Cam;         // dq VISION cam当前经过滤波姿态
    CFilterParm    m_FilterParm;                 // 滤波参数
    std::deque<CStampedPos> m_RecentPoses;
    std::deque<CStampedPos> m_RecentPoses_Cam;
    unsigned int   m_nSlideMeanSize;
    unsigned int   m_nSlideDataCount;
    unsigned int   m_nSlideDataCount_Cam;
    std::mutex     cloud_mtx;                    // 点云回调函数锁

    bool           m_bRecordImg;

    int            m_topVisionMode;              //融合顶视模式 0、不用顶视定位 1、只用顶视定位 2、激光融合顶视
    int            m_topVisionDefaultMode;

public:
    HANDLE     m_hLocKillThread;       // Handle of "Kill thread" event
    HANDLE     m_hLocThreadDead;       // Handle of "Thread dead" event
    pthread_t  m_pLocThread;
    atomic_ullong m_LocCurTime;
    atomic_ullong m_LocPreTime;
    atomic_ullong m_LocEndTime;
    bool          simulate_; //是否为仿真模式
    CScannerGroupParam ScannerParam_;      // 扫描器参数

    CStampedPos    m_PosNow;       // By Sam: 当前姿态
    bool           m_bInitLocSuccess;
    int            m_nSetPosCounter;

    // dq VISION 连续使用相机的帧数(0,20)
    float            m_CamLocCount;
    float            m_MaxCamLocCount = 20;
    bool m_reloc = false; //是否重定位
    int m_addPosTimes;
    double m_xVar;
    double m_yVar;
    double m_fThitaVar;

    double corridor_dist;

    bool            m_bFlagCameraLoc;


    std::mutex          loc_mtx;

    int   m_curFloorNo;
    bool  m_bLoadMapSuccess;
    bool m_bFirstFeatureFail = true;


private:
    CLocalizeFactory();
    friend MagicSingleton2<CLocalizeFactory>;
public:
    ~CLocalizeFactory();
    virtual bool Create();
    // 设置当前的姿态
    void SetPose(double init_x,double init_y,double init_theta,unsigned long long  init_raw_time);
    void SetPose(CStampedPos locPos);
    bool HandleSetPoseCmd();

    // by DQ
    bool LoadMap(string map_name = "FeatureMap");
    bool LoadFloorMap(string map_name = "FeatureMap",int floor=0);
    bool Initialize(bool simulate = false);
    bool StartLocalize();
    bool StopLocalize();
    bool LocalizationProc();
    // 对输入的原始点云转换成笛卡尔坐标系
    bool TransformCloud(const std::shared_ptr<sensor::CRawPointCloud>& pRawCloud, ndt_oru::CStampedPointCloud& cloud_trans);
    // 当收到激光数据时进行的回调
    void OnReceiveLaserRawScan(unsigned int nScannerId, const std::shared_ptr<sensor::CRawPointCloud>& pCloud);
    // 对计算出的结果姿态进行平滑滤波
    bool FilterPose(const CStampedPos& pose);
    // dq VISION 平滑滤波相机定位姿态
    bool FilterPose_Cam(const CStampedPos& pose);
    // 初始化参数
    bool InitializeFilterParms();
    bool GetFilteredPose(bool& filter_enable,CStampedPos& filtered_pose);
    bool GetFilteredPose_Cam(bool& filter_enable,CStampedPos& filtered_pose);
    // by DQ 添加模板
    void GetStaticObjects(int count, vector<vector<float> > points, vector<CPosture> &psts);
    bool ChangeObj(int count, vector<vector<float> > points, vector<CPosture> &psts);
    // by DQ 添加其他区域
    void GetPlans(int count, vector<vector<float> > plans, vector<int> type);
    bool ChangePlan(int plan_num, vector<vector<float> > plans, vector<int> type);
    // 定位过程函数
    virtual CMatchInfo *Localize(Eigen::Affine3d &estimatePose);

    // By Sam Add
    void CollectLaserCloud();

    //
    bool TransformRawScanToLaserMsg(const sensor::CRawScan& rawScan);
    void LocResultWriteBlackBox(int methodType, int locResult,CPosture &initPst,Eigen::Affine3d &resultPose);
    bool SetScanMatchLocArea(int area_num, const vector<vector<float> > areas, const vector<vector<int>> params);
    
    bool GetScanMatchLocArea();
    CStampedPos GetCurPose(void);

    int LegLocalize(Eigen::Affine3d &estimatePose);   // By Sam For LegMethod

    int StartRecordImage();

    int StopRecordImage();

    inline bool IsRecordingImage() { return m_bRecordImg;};


private:
    bool DealWithLocResult(const CStampedPos& pose, const CStampedPos& leg_pose,CMatchInfo* results);

    // dq VISION 处理相机和激光结果
    bool DealWithLocResult_Cam(CStampedPos& pose, const CStampedPos& leg_pose,CMatchInfo* results, const CStampedPos &pose_cam, const short errorcode_cam);
	

	
};

} // namespace robo

using LocalizeFactorySingleton = MagicSingleton2<robo::CLocalizeFactory>;

