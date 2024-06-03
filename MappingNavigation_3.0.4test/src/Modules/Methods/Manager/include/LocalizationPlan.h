#pragma once

#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"
#include "NdtLocArea.h"
#include "FeatureLocArea.h"
#include "TemplateLocArea.h"
#include "SlamLocArea.h"
#define MAX_METHODS_PER_RECT 2    // 每个定位矩形中可包含的最大定位方法数

class CLocalizationMethods;

///////////////////////////////////////////////////////////////////////////////
// 定位应用矩形区域: 用来定义在该矩形区域内的定位方案
class DllExport CLocalizationRect : public CRectangle
{
  private:
    static CLocalizationMethods *methods_;    // 指向所有定位方法集合的指针

  public:
    int methodId_[MAX_METHODS_PER_RECT];    // 定位方法编号:0-NDT; 1-特征法;2-模板法;3-栅格定位; 4-slam定位 -1-不存在
    CLocalizationParam *param_[MAX_METHODS_PER_RECT];    // 指向对应方法的参数块的指针

  public:
    CLocalizationRect();
    CLocalizationRect(const CLocalizationRect &other);
    CLocalizationRect(const CRectangle &r);

    ~CLocalizationRect();

    // 设置指向定位方法集合的指针
    static void SetLocalizationMethods(CLocalizationMethods *methods);

    // 清除所有“定位方法数据”
    void ClearMethods();

    // 判断指定的定位方法在此矩形区域中是否被启用
    int CheckMethodUsed(int index);

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
// "定位指令"类型
class DllExport CLocalizationInst
{
  public:
    int methodId_;                 // 定位方法代码(同CLocalizationRect中的定义)
    CLocalizationParam *param_;    // 指向对应的参数块的指针
    CRectangle          localizationRect_; //定位的矩形区域
  public:
    CLocalizationInst() {}

    // 根据给定的定位矩形及参数块，生成对应于“首选”或“备选”的具体定位指令
    void Create(const CLocalizationRect &r, int index);
};

///////////////////////////////////////////////////////////////////////////////
//  定义“定位方案”类型。
class DllExport CLocalizationPlan : public CAppArea
{
  private:
    CLocalizationInst instructions_[MAX_METHODS_PER_RECT];    // “定位指令”数据结构

  public:
    CLocalizationPlan() {}

    // 生成一个新的矩形应用区域
    virtual CRectangle *CreateRect() { return new CLocalizationRect; }

    // 对于给定的姿态，根据定位方案形成对应的“定位指令”
    CLocalizationInst *GetInstructions(const CPosture &pst);
};
