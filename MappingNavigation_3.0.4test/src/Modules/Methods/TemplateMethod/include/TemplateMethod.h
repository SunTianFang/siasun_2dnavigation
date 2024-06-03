#pragma once

#include "LocalizationMethod.h"
#include "StaticObjects.h"
#include "TemplateLocArea.h"
#include "Scan.h"
#include "PointFeatureSet.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板法的定位匹配信息结构。
class DllExport CTemplateMatchInfo : public CMatchInfo
{

public:
    int    nvalid;           // Number of valid correspondence in the end
    float  valid_percent;    // 合格的匹配占合格扫描线的百分比
    double error;            // 点到线的平均误差距离
    double error2;           // 点到点的平均误差距离
    double correspondencePercent;  // 覆盖率，匹配点个数 / target点数
    double histCorrelation;   // 计算target与source角度的分布相似性, 越接近1越相似

public:
    CTemplateMatchInfo() { type_ = 2; }

    // 必须提供“复制”(Duplicate)方法，以满足应用框架的需要
    virtual CMatchInfo *Duplicate() const
    {
        CTemplateMatchInfo *p = new CTemplateMatchInfo;
        *p = *this;
        return p;
    }
};

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板的定位方法
class DllExport CTemplateMethod : public CLocalizationMethod
{
  public:
    // jzz: 定义一个结构体包含每个模板对应的target,source
    struct objTargetSource
    {
        CBasObject *obj;
        CPnt centerPoint;
//        CScan *targetScan;
//        CScan *sourceScan;
    };
    vector<objTargetSource> vObjTS_;   // 存储多个特征target,source点云

    CScan *targetScan_;               // 指向目标点云的指针
    CScan *sourceScan_;               // 指向源点云的指针
    CTemplateMatchInfo matchInfo_;    // 保存每次定位的结果信息

    CScan *clusterSourceScan_;        // 指向源点云通过聚类筛选后的点云指针
    CScan *filterTargetScan_;         // 指向目标点云（筛选掉与模板非相交的点(a = 0)）的指针

    CPosture lastPose_;                // 上一帧定位结果
    CScan *lastScan_;                  // 上一帧点云
    int tolerance;                     // 容忍定位失败的次数
    CScan *viewTargetScan_;            // 用于显示用的上一帧（target)点云
    CPosture viewLastPose_;            // 用于显示用的上一帧（target)位姿

    vector<CScan> scanMap_;            // 帧间匹配得到的点云地图
//    int scanCount_;                  // 累计点云数量，用于生成scanMap_
    bool firstIn;                      // 临时测试用，记录是否是第一次进入模板匹配区域

    CPointFeatureSet *m_pPointTemplate_;// 用于显示dx中扫描到的模板点云（修正后的），用来方便绘制模板图形的位置

private:
    // 用于质量评估
    double p2lError_;                   // 点线距离，即source点到其target中对应点对线段距离
    double p2pError_;                   // 点点距离，即对应点对间距离
public:
    // 开放的质量评估参数设置
    double validPercent_;               // 匹配对点数比例（匹配点数占合格扫描线数的百分比）
    double validNum_;                   // 匹配点数个数
    // 聚类距离决定source点个数
    float clusterMaxDist_;            // sourceScan_每个点到模板中心点的最大距离 (m)
    // 此参数应该为定值
    float point2LineDist_;            // 筛选聚类的点到（组成）模板直线的距离 (m)


  protected:
    virtual CStaticObjects *CreateMap();

  public:
    CStaticObjects *map_;                 // 模板方法的地图就是静态物体集合
    CTemplateLocalizationParam param_;    // 模板法的定位参数

  public:
    CTemplateMethod();
    ~CTemplateMethod();

    // 返回指向本方法地图的指针
    CStaticObjects *GetMap() { return map_; }

    CPointFeatureSet *CreateTemplatePointsSet() {  return new CPointFeatureSet; }

    CPointFeatureSet *GetTemplatePoints() { return m_pPointTemplate_; }

    // 时间，位姿保护
    bool MotionFilter(const CPosture poseNow);

    virtual bool ReSetMethod();
    // 本方法的初始化过程
    virtual bool Initialize();

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
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将模板地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);
};
