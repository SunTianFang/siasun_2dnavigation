#pragma once

#include <vector>
#include "LocalizationMethod.h"

#define NUM_LOCALIZATION_METHODS 5    // 目前所支持的定位方法的数量(当增加新方法时应更改此宏!!)

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// 定义所有定位方法的集合
class DllExport CLocalizationMethods : public vector<CLocalizationMethod *>
{
  public:
    CLocalizationMethods() {}
    ~CLocalizationMethods() { Clear(); }

    // 清除所有定位方法
    void Clear();


    virtual bool UnloadMap(); //lishen

    // 生成所有的定位方法
    virtual bool Create();

    // 初始化所有定位方法
    virtual bool Initialize();

    // 根据给定的方法编号，生成对应的定位参数
    virtual CLocalizationParam *CreateLocalizationParam(int index);

    // 从二进制文件装入地图
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将地图写入二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);
    virtual bool SaveFeatureBinary(FILE *fp, string filename);
};
