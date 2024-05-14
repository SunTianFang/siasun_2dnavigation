#pragma once

#include <vector>
#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"



using namespace std;

///////////////////////////////////////////////////////////////////////////////
// by lishen 基于grid scan match 的定位方法所使用的参数


class DllExport CScanMatchParam : public CLocalizationParam
{
  public:


    double  m_ratioMatchWithMap;       // 匹配率：激光实际击中栅格数/应该击中栅格数
    double  m_rationMatchWithLaser ;   //匹配率：激光实际击中栅格线数/应该击中线数
    double  m_linearSearchWindow ;     //分支限界距离搜索范围
    double  m_angularSearchWindow ;    // 分支限界角度搜索范围
    double  m_reloc_linearSearchWindow ;     // 重定位分支限界距离搜索范围
    double  m_reloc_angularSearchWindow ;    // 重定位分支限界角度搜索范围
    double  m_scoreFastMatch;          //分支限界成功最小成绩
    int  m_localMode;              // 0-正常模式 1-单点模式x 2-单点模式y 3-长廊模式x 4-长廊模式y

    int  m_topVisionMode;

    double  m_bUseSingleFeature;

  public:
    CScanMatchParam();
    CScanMatchParam(const CScanMatchParam &other);

    // 生成本数据的一个副本
    virtual CScanMatchParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};


///////////////////////////////////////////////////////////////////////////////
// Grid Scanmatch定位应用矩形区域: 定义在该矩形区域内的Grid Scanmatch定位参数
class DllExport CScanMatchLocalizationRect : public CRectangle
{
  public:
    CScanMatchParam param_;    // Grid Scanmatch方法所使用的参数

  public:
    CScanMatchLocalizationRect() {}

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
