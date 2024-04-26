#include <stdafx.h>
#include "SlamLocArea.h"

///////////////////////////////////////////////////////////////////////////////

CSlamLocalizationParam::CSlamLocalizationParam()
{
    SlamMaxDis = 20;     //允许单次slam最远距离
    SlamTotalDis = 20;  //允许单次slam最长距离
   // m_ratioMatchWithMap = 0.2;
   // m_rationMatchWithLaser = 0.2;
   // m_linearSearchWindow = 0.5;
   // m_angularSearchWindow =25.0*PI/180.0;
   // m_scoreFastMatch = 0.3;
}

CSlamLocalizationParam::CSlamLocalizationParam(const CSlamLocalizationParam &other)
{
    *this = other;
}

//
//   生成本数据的一个副本。
//
CLocalizationParam *CSlamLocalizationParam::Duplicate()
{
    CSlamLocalizationParam *copy = new CSlamLocalizationParam;
    *copy = *this;
    return copy;
}

// by dq
bool CSlamLocalizationParam::LoadBinary(FILE *fp)
{
    if (fread(&SlamMaxDis, sizeof(float), 1, fp) != 1)
        return false;
    if (fread(&SlamTotalDis, sizeof(float), 1, fp) != 1)
        return false;

    return true;
}
//by dq
bool CSlamLocalizationParam::SaveBinary(FILE *fp)
{
    // 写入最大允许SLAM定位距离
    if (fwrite(&SlamMaxDis, sizeof(float), 1, fp) != 1)
        return false;
    if (fwrite(&SlamTotalDis, sizeof(float), 1, fp) != 1)
        return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   NDT定位应用矩形: 定义在该矩形区域内的NDT定位参数。

//
//   从二进制文件读取数据。
//
bool CSlamLocalizationRect::LoadBinary(FILE *fp)
{
    if (!CRectangle::LoadBinary(fp))
        return false;

    return param_.LoadBinary(fp);
}

//
//   将数据写入二进制文件。
//
bool CSlamLocalizationRect::SaveBinary(FILE *fp)
{
    if (!CRectangle::SaveBinary(fp))
        return false;

    return param_.SaveBinary(fp);
}
