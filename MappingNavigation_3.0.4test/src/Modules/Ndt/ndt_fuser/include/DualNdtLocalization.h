#pragma once

#include <Eigen/Core>
#include <mutex>
#include "Geometry.h"
//#include "Mapping/BaseOdometry.h"
#include "ScannerParam.h"
#include "RawScan.h"
#include "NdtBaseLocalization_oru.h"
#include "NdtLocalization_omp.h"
#include "StampedPos.h"
//#include "LocalizationPlan.h"

#define LOC_SLIDE_MEAN_SIZE 4

// 匹配点数，匹配率%
#define NDT_THRESH_COUNT 5
#define NDT_1_THRESH_RATE 0.3
#define NDT_2_THRESH_RATE 0.45

#define USE_CORRODOR_FUNCTION 1    // 1 -> use; 0 -> not use
#define CORRIDOR_DIS_LIMIT 20000

//
//   定义具有时间戳的姿态。
//
class CStampedAffine : public Eigen::Affine3d, public CTimeStamp
{
  public:
    CStampedAffine(Eigen::Affine3d &affine, CTimeStamp &stamp) : Eigen::Affine3d(affine), CTimeStamp(stamp) {}

    CStampedAffine(Eigen::Affine3d &affine, unsigned int uTime) : Eigen::Affine3d(affine), CTimeStamp(uTime) {}

    CStampedAffine(Eigen::Affine3d &affine) : Eigen::Affine3d(affine) {}

    CStampedAffine() {}

    Eigen::Affine3d &GetAffine3dObject()
    {
        Eigen::Affine3d *p = static_cast<Eigen::Affine3d *>(this);
        return *p;
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

    ~CFilterParm() {}

    CFilterParm(const CFilterParm &other)
    {
        this->vFactor.clear();
        this->bEnable = other.bEnable;
        this->vFactor = other.vFactor;
    }

    CFilterParm &operator=(const CFilterParm &Obj)
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

class CRectRegion : public CRectangle
{
  public:
    CRectRegion(float x1 = 0, float y1 = 0, float x2 = 0, float y2 = 0, int type = 0, int uN = 0) :
        CRectangle(x1, y1, x2, y2)
    {
        regionType = type;
        regionUN = uN;
    }

  public:
    int regionType;
    int regionUN;
};

class CRegionPlan : public std::vector<CRectRegion>
{
  public:
    void save(FILE *jffout)
    {
        int n = size();
        fwrite(&n, sizeof(int), 1, jffout);
        for (int i = 0; i < size(); i++)
            fwrite(&at(i), sizeof(CRectRegion), 1, jffout);
    }

    int load(FILE *jffin)
    {
        int n(0);
        if (fread(&n, sizeof(int), 1, jffin) != 1)
            return -1;
        clear();
        reserve(n);
        for (int i = 0; i < n; i++)
        {
            CRectRegion tmpRegion;

            if (fread(&tmpRegion, sizeof(CRectRegion), 1, jffin) != 1)
                return -1;
            push_back(tmpRegion);
        }
        return n;
    }

    int loadDump(FILE *jffin)
    {
        int n(0);
        float f[4];
        int type_temp;
        if (fread(&n, sizeof(int), 1, jffin) != 1)
            return 0;
        for (int i = 0; i < n; i++)
        {
            fread(&type_temp, sizeof(int), 1, jffin);

            if (type_temp != 1)
                fread(f, sizeof(float), 2, jffin);
            else
                fread(f, sizeof(float), 3, jffin);
        }
        if (fread(&n, sizeof(int), 1, jffin) != 1)
            return 0;
        for (int i = 0; i < n; i++)
        {
            if (fread(f, sizeof(float), 4, jffin) != 4)
                return 0;
            if (fread(&type_temp, sizeof(int), 1, jffin) != 1)
                return 0;
        }
        if (fread(&n, sizeof(int), 1, jffin) != 1)
            return 0;
        for (int i = 0; i < n; i++)
        {
            if (fread(&type_temp, sizeof(int), 1, jffin) != 1)
                return 0;
            if (fread(f, sizeof(float), 4, jffin) != 4)
                return 0;
        }
        return n;
    }
};

//////////////////////////////////////////////////////////////////////////////
//   适用于多激光扫描器的定NDT定位。

///////////////////////////////////////////////////////////////////////////////
//
//   CDualNdtLocalization封装了对具有多个激光器的机器人进行定位的方法。
//
//   说明：
//      1. 机器人可以拥有多个激光器(各自有不同的安装姿态)
//      2. 定位只针对机器人本体，不针对任何一个激光器
//      3. 里程数据、激光扫描数据都具有时间戳，并且它们可以异步到来
//      4. 定位算法考虑了机器人的速度因素，利用时间戳进行了补偿
//
///////////////////////////////////////////////////////////////////////////////
class CDualNdtLocalization
{
  protected:
    // 处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补
    virtual void CollectOdomLaserData(long long int tmStart);

    // 对计算出的结果姿态进行平滑滤波
    bool FilterPose(const CStampedPos &pose);

  private:
    bool corridorJudge(const Eigen::Affine3d &pos);

    bool onlyUseNdt_1(NDTMatchInfo &ndtInfo, int &uN);

    bool ProcessNdt1Only(Eigen::Affine3d &Testimate_oru, NDTMatchInfo &info_oru);

    bool ProcessStopCondition(unsigned int tmStart, CPosture &estmPos);

    Eigen::Affine3d GetTransform(unsigned int tmLast, unsigned tmNow);

    bool ProcessCorridorCondition(const Eigen::Affine3d &Tinit, Eigen::Affine3d &Testimate_oru, NDTMatchInfo &info_oru);

  public:
    CDualNdtLocalization(double map_reso = DEFAULT_MAP_RESO);

    // 设置激光扫描器组参数
    void SetScannerGroupParam(CScannerGroupParam *pParam);

    // 设置定位初始姿态
    virtual void SetPose(double x, double y, double thita, unsigned long long time_stamp);

    // 当收到里程变化数据时进行的回调
    void OnReceiveOdometry(const CStampedPos &odomMove);

    void OnReceiveLaserScan(unsigned int nScannerId, const std::shared_ptr<sensor::CRawPointCloud> &pCloud);

    // 进行异步的定位并返回姿态结果
    virtual bool AsynLocalize(CStampedPos &pose, NDTMatchInfo &info_omp, NDTMatchInfo &info_oru,
                              bool realTimeRunning = true);

    // in: x, y;   out: uN
    int inRegion(double &x, double &y, int &uN);

    // 载入地图
    bool LoadMap(FILE *fp);

    // 初始化参数
    bool InitializeParms();

    bool InitNDTParms();

    bool GetFilteredPose(bool &filter_enable, CStampedPos &filtered_pose);

    // 是否进入扩展定位
    bool EnterLocalizeEx();

    // 在指定的范围内进行扩展定位(静态定位)
    virtual bool LocalizeEx(ndt_oru::CPointCloud &cloud_in, Eigen::Affine3d &odometry, Eigen::Affine3d &estimate);

  private:
    std::mutex odometry_mtx;    // 里程计回调函数锁
    std::mutex cloud_mtx;       // 点云回调函数锁

    CStampedPos m_PosNowFiltered;    // 当前经过滤波姿态
    CFilterParm m_FilterParm;        // 滤波参数
    std::deque<CStampedPos> m_RecentPoses;
    unsigned int m_nSlideMeanSize;
    unsigned int m_nSlideDataCount;

    bool m_bUseExInitPose;       //使用扩展定位在设置位姿
    bool m_bUseExPathTracing;    //使用扩展定位在路径跟踪
    bool m_bEnterLocalizeEx;     //扩展定位模式

  public:
    CRegionPlan regions;
    //    featureSet
    // ova: original version     omp: robust version
    ndt_oru::CNdtBaseLocalization *mpNdtSolver_oru;
    ndt_omp::CNdtLocalization *mpNdtSolver_omp;

    double corridor_dist;
    bool corridor_firstTime;
    //    unsigned long long corridor_timeNow;
    unsigned long long corridor_timeBefore;    // 上一次检测到处于过道的时间

    // AGV stop variable
    int m_addPosTimes;
    double m_xVar;
    double m_yVar;
    double m_fThitaVar;

    // NDT param
    int m_ndtThreshNum;
    double m_ndt1ThreshRate;
    double m_ndt2ThreshRate;

    bool m_corridorFunction;
    int m_corridorDistanceLimit;

    bool m_onlyNdt1Function;
    int m_minNumLimit;

    std::vector<CRectRegion> RegionType_RL;    // 读取区域

  protected:
    CStampedPos m_PosNow;                                 // 当前姿态
    CScannerGroupParam m_ScannerGroupParam;               // 激光器组参数
    std::vector<ndt_oru::CLabeledPointCloud> m_clouds;    // 获得的所有点云
    ndt_oru::CStampedPointCloud m_CloudAdjusted;          // 汇总后的点云
    CStampedPos m_OdomMove;                               // 未经插补的里程姿态
    Eigen::Affine3d m_OdomMoveAdjusted;                   // 插补之后得到的里程姿态
    Eigen::Affine3d m_Velo;                               // 速度向量

    bool m_bSetPose;               //设置初始位姿
    unsigned int m_nLocFailCnt;    //正常定位失败次数
};
