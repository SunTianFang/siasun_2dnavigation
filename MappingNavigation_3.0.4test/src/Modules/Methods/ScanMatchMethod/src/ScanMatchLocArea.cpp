#include <stdafx.h>
#include "ScanMatchLocArea.h"

///////////////////////////////////////////////////////////////////////////////

//
//   By lishen : scan match param
//

CScanMatchParam::CScanMatchParam()
{
    m_ratioMatchWithMap = 0.2;      //
    m_rationMatchWithLaser = 0.2;  //
    m_linearSearchWindow = 0.5;
    m_angularSearchWindow =25.0*PI/180.0;
    m_scoreFastMatch = 0.3;
    m_localMode= 0;
    m_bUseSingleFeature = false;
    m_reloc_linearSearchWindow =  2;     // 重定位分支限界距离搜索范围
    m_reloc_angularSearchWindow = 60.0*PI/180.0;;    // 重定位分支限界角度搜索范围

    m_topVisionMode = 0;



}

CScanMatchParam::CScanMatchParam(const CScanMatchParam &other)
{
    *this = other;
}

CScanMatchParam *CScanMatchParam::Duplicate()
{
    CScanMatchParam *copy = new CScanMatchParam;
    *copy = *this;
    return copy;
}

bool CScanMatchParam::LoadBinary(FILE *fp)
{
    int m;
    if (fread(&m, sizeof(int), 1, fp) != 1)
        return false;
    //m_bUseSingleFeature = (bool)m;


    m_localMode = (int)(m%10) ;

    m_topVisionMode = (m- m_localMode)/10;

    return true;
}

bool CScanMatchParam::SaveBinary(FILE *fp)
{

    int m = m_localMode+m_topVisionMode*10;

    if (fwrite(&m, sizeof(int), 1, fp) != 1)
        return false;


    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   NDT定位应用矩形: 定义在该矩形区域内的NDT定位参数。

//
//   从二进制文件读取数据。
//
bool CScanMatchLocalizationRect::LoadBinary(FILE *fp)
{
    if (!CRectangle::LoadBinary(fp))
        return false;

    return param_.LoadBinary(fp);
}

//
//   将数据写入二进制文件。
//
bool CScanMatchLocalizationRect::SaveBinary(FILE *fp)
{
    if (!CRectangle::SaveBinary(fp))
        return false;

    return param_.SaveBinary(fp);
}
