#pragma once

#include "ZTypes.h"

class DllExport CLocalizationParam
{
  public:
//    int methodId_;    // 对应的定位方法的编号

  public:
    virtual CLocalizationParam *Duplicate() = 0;
    virtual bool LoadBinary(FILE *fp) = 0;
    virtual bool SaveBinary(FILE *fp) = 0;
};
