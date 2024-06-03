#pragma once

#include <vector>
#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"



using namespace std;

///////////////////////////////////////////////////////////////////////////////
// 基于NDT的定位方法所使用的参数
class DllExport CNdtLocalizationParam : public CLocalizationParam
{
  public:
    int submapId;          // 对应的NDT子图的ID号
    int localMode;         // NDT定位方式 (正常定位 -- 0   长廊定位 -- 1   局部定位 -- 2)
    bool enableSlam;       // 是否允许以SLAM方式进行定位
    float maxSlamDist;     // 允许以SLAM模式运动的最大距离
    float slamRatio;        // SLAM定位时的匹配率
    int threshNumX;        // X方向的匹配个数阈值（个） 默认值 20
    int threshNumY;        // Y方向的匹配个数阈值（个） 默认值 20
    float threshRatioX;    // X方向的匹配效率阈值（%） 默认值 40.0
    float threshRatioY;    // Y方向的匹配效率阈值（%） 默认值 40.0
    float disLimit;        // 功能启用的距离限制（米） 默认值 10.0
    int N_;                // 定位评价因子
    bool enableGridMatch;   //是否允许以栅格方式进行定位
    int gridMatchType;      //GRID定位类型

    float fastMatchLineSearchWindow;   //扩展定位XY方向匹配范围
    float fastMatchAngleSearchWindow; //扩展定位角度匹配范围


  public:
    CNdtLocalizationParam();
    CNdtLocalizationParam(const CNdtLocalizationParam &other);

    // 生成本数据的一个副本
    virtual CLocalizationParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
// NDT定位应用矩形区域: 定义在该矩形区域内的NDT定位参数
class DllExport CNdtLocalizationRect : public CRectangle
{
  public:
    CNdtLocalizationParam param_;    // ndt方法所使用的参数

  public:
    CNdtLocalizationRect() {}

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
