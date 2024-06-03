#pragma once

#include "ZTypes.h"
#include <stdio.h>
#include "Eigen/Eigen"
#include "ndt_pointcloud.h"
#include "LocalizationPlan.h"
#include "MatchInfo.h"
#include "ScannerParam.h"

class DllExport CLocalizationMethod
{
  public:
    int type_;
    CScannerGroupParam *m_pScannerGroupParam;    // 激光器组参数
    CRectangle          *localizationRect_; //定位的矩形区域
  public:
    CLocalizationMethod() { m_pScannerGroupParam = NULL; localizationRect_ = NULL ; type_ = -1; }
    virtual ~CLocalizationMethod() {}

    virtual bool Initialize() = 0;

    virtual bool UnloadMap() = 0; //lishen

    // 生成一个适用于本方法的定位参数块
    virtual CLocalizationParam *CreateLocParam() = 0;

    virtual void SetScannerGroupParam(CScannerGroupParam *pParam){m_pScannerGroupParam = pParam;}

    // 应用指定的定位参数
    virtual bool ApplyParam(const CLocalizationParam *param) = 0;

    // 应用指定的定位参数
#pragma GCC push_options
#pragma GCC optimize ("O0")
   virtual bool ApplyLocRect(CRectangle *r){localizationRect_ = r;}
#pragma GCC pop_options

    // 定位方法的定位流程
    virtual bool LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                              const ndt_oru::CStampedPointCloud cloudIn,
                              Eigen::Affine3d &estimatePose) = 0;

    // 取得匹配数据
    virtual CMatchInfo *GetMatchInfo() = 0;

    // 对定位质量进行评估
    virtual bool EvaluateQuality(float &score) = 0;

    // 从二进制文件装入地图
   // virtual bool LoadBinary(FILE *fp, string filename/*,int floor*/) = 0;

    // 从二进制文件装入地图
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor) = 0;

    // 将地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename) = 0;

    // By Sam 
    virtual bool ReSetMethod() = 0;
};
