#pragma once

#include "LocalizationMethod.h"
//#include "StaticObjects.h"
#include "Scan.h"
#include "real_time_correlative_scan_matcher_2d.h"
//#include "fast_correlative_scan_matcher_2d.h"
#include "probability_grid.h"
//#include "ndt_maps_oru.h"
//#include "ndt_pointcloud.h"
#include "make_unique.h"
//#include "ceres_scan_matcher_2d.h"
//#include "FastMatchMethod.h"

#include "navigation.h"
#include "SlamLocArea.h"


////////////////////////////////////////////////
//   实现栅格slam的定位方法
//   Author: lishen
//   Date:   2022.10.30
///////////////////////////////////////////////


using namespace mapping;
using namespace sensor;
#if 0
class DllExport CSlamParam : public CLocalizationParam
{
  public:


    double  m_ratioMatchWithMap;       //匹配率：激光实际击中栅格数/应该击中栅格数
    double  m_rationMatchWithLaser ;   //匹配率：激光实际击中栅格线数/应该击中线数
    double  m_linearSearchWindow ;     //分支限界距离搜索范围
    double  m_angularSearchWindow ;    //分支限界角度搜索范围
    double  m_scoreFastMatch;          //分支限界成功最小成绩
    bool    m_bCorridorFlag;
    double  m_linearSearchWindowOutCorrior;
    double  m_minCorridorLength;
    double  m_minLineLength;

  public:
    CSlamParam();
    CSlamParam(const CSlamParam &other);

    // 生成本数据的一个副本
    virtual CSlamParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
#endif
class DllExport CSlamMatchInfo : public CMatchInfo
{
  public:

    CPosture initposture;
    ndt_oru::CStampedPointCloud cloudIn;

  public:
    CSlamMatchInfo() { type_ = 4; }


    // 必须提供“复制”(Duplicate)方法，以满足应用框架的需要
    virtual CMatchInfo *Duplicate() const
    {
        CSlamMatchInfo *p = new CSlamMatchInfo;
        *p = *this;
        return p;
    }
};

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板的定位方法
class DllExport CSlamMethod : public CLocalizationMethod
{

  private:

    float       m_score;                  //匹配率
    bool        m_locOK;                  //定位是否成功
    CPosture    m_lastRobotPos;       //上周期定位结果
    CPosture    m_posSlamResult;
    CSlamLocalizationParam  m_param;
    CCartoSlam          *m_pCartoSlam;

    bool        m_bSlaming;
    double      m_SlamMaxDis;
    double      m_SlamTotalDis;
    CPosture    local_pst;
    CPosture    lastlocal_pst;
    CPosture    orilocal_pst;
    double      m_realtimescore;
    bool        m_res;


    Eigen::Affine3d m_lastPos;
    laserscan_msg   m_lastScanMsg;
    odometry_msg    m_lastOdomMsg;

 public:

    laserscan_msg   m_scanMsg;
    odometry_msg    m_odomMsg;


    int         m_SlamCount;

    CSlamMatchInfo matchInfo_;                          // 保存每次定位的结果信息

   // std::unique_ptr<ProbabilityGrid> map_;           // 概率地图
   //  by lishen
    void SetLastPos(unsigned long long laststamp, Eigen::Affine3d lpos);
    //  by lishen
    static void CartoSlamCallback(slam_result *pSlamResult, std::vector<opt_result> *pOptResult);

    //carto创建地图结束回调函数
    static void SlamOverCallback();

     // by lishen
    void SlamCallbackFunc(slam_result *pSlamResult, std::vector<opt_result> *pOptResult);
     // by lishen
    void StopSlamLocate();

    //void PutSensorData(laserscan_msg &lscan, odometry_msg &odom);
     // by lishen
    void SetInitNodePose(const Eigen::Affine3d &initPose,unsigned long long  timestamp);

    //by dq
    void SetUseSlamMethod(bool use_slam)
    {
        m_bSlaming = use_slam;
    }
    bool GetUseSlamMethod()
    {
        return m_bSlaming;
    }
    // by lishen
    void Clear();

     // by lishen
    void InitLocate();

    // by dq
    bool UseSlam(CPosture &posSlamResult);
     // by dq
    void ResetOdom();
     // by dq
    bool ChangeToScanMatchMethod(int methodId, const bool bInitLocSuccess);

    bool IsChangeToOtherMethod();


  public:
    CSlamMethod();
    ~CSlamMethod();



    //卸载地图
    virtual bool UnloadMap();

    // 本方法的初始化过程
    virtual bool Initialize();

    // 生成一个适用于本方法的定位参数块
    virtual CLocalizationParam *CreateLocParam();

    // 应用指定的定位参数
    virtual bool ApplyParam(const CLocalizationParam *param);

    // 对应于该定位方法的定位流程
    virtual bool LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                              const ndt_oru::CStampedPointCloud cloudIn,
                              Eigen::Affine3d &estimatePose);

    // 取得匹配数据
    virtual CMatchInfo *GetMatchInfo();

    // 对定位质量进行评估
    virtual bool EvaluateQuality(float &score);

    // 从二进制文件装入模板地图
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将模板地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);

    // By Sam 
    virtual bool ReSetMethod();

};
