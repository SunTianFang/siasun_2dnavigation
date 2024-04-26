#pragma once

#include <vector>
#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"



using namespace std;

///////////////////////////////////////////////////////////////////////////////
// by dq 基于grid slam的定位方法所使用的参数


class DllExport CSlamLocalizationParam : public CLocalizationParam
{
  public:
    float SlamMaxDis;   //允许单次slam最远距离
    float SlamTotalDis; //允许单次slam最长距离


  public:
    CSlamLocalizationParam();
    CSlamLocalizationParam(const CSlamLocalizationParam &other);

    // 生成本数据的一个副本
    virtual CLocalizationParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
// Slam定位应用矩形区域: 定义在该矩形区域内的Slam定位参数
class DllExport CSlamLocalizationRect : public CRectangle
{
  public:
    CSlamLocalizationParam param_;    // Slam方法所使用的参数

  public:
    CSlamLocalizationRect() {}

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
