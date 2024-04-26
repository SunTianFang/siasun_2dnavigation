#pragma once

#include "LocalizationMethod.h"
#include "ndt_maps_oru.h"
#include "NdtLocArea.h"
#include "NdtExtLocalization_oru.h"


#include "FastMatchMethod.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于NDT的定位方法
class DllExport CNdtMethod : public CLocalizationMethod
{
  public:
    ndt_oru::NDTMaps *maps_;
    CNdtLocalizationParam param_;
    ndt_oru::CNdtExtLocalization *localization_;

    bool m_bInitSlam;    // By Sam: 当前是否处于SLAM定位状态

    CFastMatchMethod m_fastMatcher;


  public:
    CNdtMethod();
    ~CNdtMethod();

    virtual bool Create();

    ndt_oru::NDTMaps &GetMaps() { return *maps_; }

    // 取得当前所采用的NDT子图的编号
    int GetCurSubmapId();

    virtual bool Initialize();

    virtual bool UnloadMap();

    // 生成一个适用于本方法的定位参数块
    virtual CLocalizationParam *CreateLocParam();

    // 应用指定的定位参数
    virtual bool ApplyParam(const CLocalizationParam *param);

    virtual void SetScannerGroupParam(CScannerGroupParam *pParam);

    // 对应于该定位方法的定位流程
    virtual bool LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                              const ndt_oru::CStampedPointCloud cloudIn,
                              Eigen::Affine3d &estimatePose);

    // 取得匹配数据
    virtual CMatchInfo *GetMatchInfo();

    // 对定位质量进行评估
    virtual bool EvaluateQuality(float &score);

    // 从二进制文件装入NDT地图
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将NDT地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);

    // By Sam 
    virtual bool ReSetMethod();

    bool CorridorJudge(float x, float y);
public:
    double corridor_dist;
    bool  corridor_firstTime;
//    unsigned long long corridor_timeNow;
    unsigned long long corridor_timeBefore;
    bool m_corridorFunction;
    int m_corridorDistanceLimit;
};
