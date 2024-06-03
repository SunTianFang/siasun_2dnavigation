#pragma once

#include <vector>
#include <string>
#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// 基于模板的定位方法所使用的参数
class DllExport CTemplateLocalizationParam : public CLocalizationParam
{
  public:
    int objId;               // 匹配物体的编号
    vector<string> objNames; // 待匹配物体的名称集合
    float directionAngle;    // 匹配点与虚拟扫描点的点向量方向的夹角（deg）默认值30.0
//    int templateNum;         // 模板形状匹配个数，匹配到的货架腿的个数 默认值 2
    int useMultiTemplate;     // 是否使用多模板共同定位，默认值 0（不启用）:同一矩形区域中，每次定位target选取一个模板，还是多模板共同生成target点云
    float templateRatio;     // 单个模板形状的匹配率 默认值 60.0

    int useTemplateNum;      // 使用匹配的模板个数，默认值 1

  public:
    CTemplateLocalizationParam();

    // 生成本数据的一个副本
    virtual CLocalizationParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
// 模板定位应用矩形区域: 定义在该矩形区域内的模板定位参数
class DllExport CTemplateLocalizationRect : public CRectangle
{
  public:
    CTemplateLocalizationParam param_;    // ndt方法所使用的参数

  public:
    CTemplateLocalizationRect() {}

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
