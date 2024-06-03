#pragma once

#include "LocalizationMethod.h"
#include "StaticObjects.h"
#include "Scan.h"
#include "real_time_correlative_scan_matcher_2d.h"
#include "fast_correlative_scan_matcher_2d.h"
#include "probability_grid.h"
#include "ndt_maps_oru.h"
#include "ndt_pointcloud.h"
#include "make_unique.h"
#include "ceres_scan_matcher_2d.h"
#include "FastMatchMethod.h"
#include "ScanMatchLocArea.h"

////////////////////////////////////////////////
//   实现基于栅格的定位方法
//   采用了cartographer中realtime 和cere定位方法实现基于栅格地图的定位
//   Author: lishen
//   Date:   2022. 6.
///////////////////////////////////////////////


using namespace mapping;
using namespace sensor;


class DllExport CScanMatchMatchInfo : public CMatchInfo
{
  public:

    CPosture initposture;
    ndt_oru::CStampedPointCloud cloudIn;

  public:
    CScanMatchMatchInfo() { type_ = 3; }


    // 必须提供“复制”(Duplicate)方法，以满足应用框架的需要
    virtual CMatchInfo *Duplicate() const
    {
        CScanMatchMatchInfo *p = new CScanMatchMatchInfo;
        *p = *this;
        return p;
    }
};

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板的定位方法
class DllExport CScanMatchMethod : public CLocalizationMethod
{

  private:

    float   m_score;                  //匹配率
    bool    m_locOK;                  //定位是否成功
    int     m_countFailed;            //定位失败后开始计数   大于几个周期调用分支限界
    CPosture    m_lastRobotPos;       //上周期定位结果
    CScanMatchParam m_param;

    scan_matching::CeresScanMatcher2D cerematcher;  //cere定位方法
    CFastMatchMethod *m_pFastMatcher;               //分支限界定位方法
    bool m_SetPos;


 public:

    CScanMatchMatchInfo matchInfo_;                  // 保存每次定位的结果信息
    std::shared_ptr<ProbabilityGrid> map_;           // 概率地图


  public:
    CScanMatchMethod();
    ~CScanMatchMethod();

    //给分支限界指针赋值，    分支限界实例及地图的加载由ndt方法中实现
    void SetFastMatch(CFastMatchMethod *p);

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

    // by dq
    void SetPose_byPad(bool setpose_bypad)
    {
        m_SetPos = setpose_bypad;
    }
    // by dq
    bool GetPose_byPad()
    {
        return m_SetPos;
    }
};
