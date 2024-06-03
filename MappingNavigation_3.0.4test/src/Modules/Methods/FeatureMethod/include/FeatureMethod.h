#pragma once

#include "LocalizationMethod.h"
#include "FeatureMap.h"
#include "FeatureLocArea.h"
#include <Eigen/Eigen>
#include "FeatureLocalization.h"
#include "FeatureMatchInfo.h"
#include "Scan.h"
#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于“特征”的定位方法。
class DllExport CFeatureMethod : public CLocalizationMethod
{
  public:
    CFeatureMatchInfo matchInfo_;
    CFeatureMap *map_;                   // 特征地图
    CFeatureLocalizationParam* param_;    // 定位应用参数
    int   criteritonThreshold_;          // 反光板识别强度门限
  public:
    CFeatureMethod();
    ~CFeatureMethod();

    // 取得特征地图
    CFeatureMap *GetMap() { return map_; }

    // 执行此定位方法的初始化
    virtual bool Initialize();

    virtual bool UnloadMap();

    // 生成一个适用于本方法的定位参数块
    virtual CLocalizationParam *CreateLocParam();

    // 应用指定的定位参数
    virtual bool ApplyParam(const CLocalizationParam *param);

    // 从二进制文件装入特征地图
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将特征地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);

    // 定位函数
    // @Param localMode 输入定位模式
    // @Param initPose  输入初始位置
    // @Param cloudIn   输入点云
    // @Param estimatePose  输出定位位置
    virtual bool LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                              const ndt_oru::CStampedPointCloud cloudIn,
                              Eigen::Affine3d &estimatePose);

    // 取得匹配数据
    virtual CMatchInfo *GetMatchInfo();

    // 对定位质量进行评估
    virtual bool EvaluateQuality(float &score);

    // By Sam
    bool GetLocParam();
    virtual bool ReSetMethod();

  protected:
    virtual CFeatureMap *CreateMap();

  private:
    //将点云生成scan
    CFeatureScan *CreatScanFromPointCloud(const ndt_oru::CStampedPointCloud &cloudIn,const bool bIsOnlyRefletor=false); //wt_fixtest_20230203

  private:
    CFeatureLocalization FeatureLocalization_;    //点线定位方法
    CFeatureMap FeatureMap;
    int localMode_;
    CPosture pstNew_;
    Eigen::Affine3d initPose_;
};
