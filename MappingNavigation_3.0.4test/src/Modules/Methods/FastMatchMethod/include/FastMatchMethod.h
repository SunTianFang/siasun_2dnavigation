#pragma once

#include "LocalizationMethod.h"
#include "StaticObjects.h"

#include "Scan.h"

#include "fast_correlative_scan_matcher_2d.h"
#include "probability_grid.h"

#include "ndt_maps_oru.h"
#include "ndt_pointcloud.h"
#include "make_unique.h"

////////////////////////////////////////////////
//   分支限界 实现大范围定位
//   Author: lishen
//   Date:   2022. 1.
///////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//   定义基于的定位匹配信息结构。

enum emGridMatchType
{
    FAST_MATCH,
    CERE_MATCH,

};

class DllExport CFastMatchParam : public CLocalizationParam
{
  public:

    float linearSearchWindow;
    float angularSearchWindow ;



  public:
    CFastMatchParam();
    CFastMatchParam(const CFastMatchParam &other);



    // 生成本数据的一个副本
    virtual CFastMatchParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

class DllExport CFastMatchMatchInfo : public CMatchInfo
{
  public:

    CPosture initposture;
    ndt_oru::CStampedPointCloud cloudIn;

  public:
    CFastMatchMatchInfo() { type_ = 2; }


    // 必须提供“复制”(Duplicate)方法，以满足应用框架的需要
    virtual CMatchInfo *Duplicate() const
    {
        CFastMatchMatchInfo *p = new CFastMatchMatchInfo;
        *p = *this;
        return p;
    }
};

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板的定位方法
class DllExport CFastMatchMethod : public CLocalizationMethod
{

  public:

    CFastMatchMatchInfo matchInfo_;    // 保存每次定位的结果信息


    std::map<int, std::shared_ptr<mapping::scan_matching::FastCorrelativeScanMatcher2D>> matchers_;  //first 子图ID

  private:

    int     m_curSubmapId;            //当前子图ID
    float   m_score;                 //
    double  m_linearSearchWindow;     //
    double  m_angularSearchWindow ;   //


  public:

  //  CFastMatchParam param_;    // 定位参数

  public:
    CFastMatchMethod();
    ~CFastMatchMethod();


    // by lishen  由ndt地图生成分支限界用的地图  现在不在用了，直接由栅格地图生成了
    bool CreateGridMap(ndt_oru::NDTMaps *ndtMaps,FILE *fp);
     // by lishen
    void SetParam(float linear_search_window,float angular_search_window);
     // by lishen
    void SetMapID(int submapID);

    // 本方法的初始化过程
    virtual bool Initialize();

    // by lishen
    virtual bool UnloadMap();

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
    virtual bool LoadBinary(FILE *fpp, string filename,int floor, bool bChangeFloor);

    // 将模板地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);

    // By Sam 
    virtual bool ReSetMethod();
};
