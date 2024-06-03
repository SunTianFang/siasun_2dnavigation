#include <stdafx.h>
#include "FeatureLocArea.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于特征的定位参数。

CFeatureLocalizationParam::CFeatureLocalizationParam()
{
    ratio = 60;
    minRange = 0.1f;
    maxRange = 40.0f;
    maxRefLineRange  = maxRange / 3.0; //最大的直线参考集范围
    nLeastMatchCount_ = 3;
    nMostMatchCount_ = 5;
    fSamePointMaxDist_ = 0.1f;
    angleEqualLimit = CAngle::ToRadian(5);
    rangeEqualLimit = 0.3f;
    criteritonThreshold = 254; //反光板强度阈值
    refEfficientPointCount = 3;
    fLineEqualLimit_[0] = 0.01f;
    fLineEqualLimit_[1] = 0.02f;
    fLineEqualLimit_[2] = 0.05f;
    fLineEqualLimit_[3] = 0.08f;
    fLineEqualLimit_[4] = 0.1f;
    fMinLineLen_ = 0.30f; // 最短直线特征长度
    vecSpecialPntList_.clear();
    vecSpecialLineList_.clear();
    onlyUseInRectFeature = false;
    isUseUseMultiFrameLoc = true; //是否只应用多帧定位方法
    iMultiFrameLocDequeSize = 3; //多帧定位时应用的帧数
    fMaDisPowerfulValue = 0.13; //马氏距离阀值
    isUseSinglePointLoc = false; //是否应用单点定位方法
    m_MaxDisWithoutFeature = 0.5;
}

//
//   生成本数据的一个副本。
//
CLocalizationParam *CFeatureLocalizationParam::Duplicate()
{
    CFeatureLocalizationParam *copy = new CFeatureLocalizationParam;
    *copy = *this;
    return copy;
}

bool CFeatureLocalizationParam::LoadBinary(FILE *fp)
{
#if 0
    if (fread(&maxRange, sizeof(int), 1, fp) != 1)
        return false;
#else
    float f[2];
    if (fread(f, sizeof(float), 2, fp) != 2)
        return false;

    minRange = f[0];
    maxRange = f[1];

    if (fread(&nMostMatchCount_, sizeof(int), 1, fp) != 1)
        return false;

    if (fread(f, sizeof(float), 2, fp) != 2)
        return false;

    angleEqualLimit = f[0];
    rangeEqualLimit = f[1];

    int n[2];
    if (fread(n, sizeof(int), 2, fp) != 2)
        return false;

    criteritonThreshold = n[0];
    refEfficientPointCount = n[1];

    if (fread(&ratio, sizeof(float), 1, fp) != 1)
        return false;


    int inPoint;
    if (fread(&inPoint, sizeof(int), 1, fp) != 1)
        return false;
    for(int i = 0 ; i < inPoint ; i ++)
    {
        CPnt t;
        t.LoadBinary(fp);
        vecSpecialPntList_.push_back(t);
    }

    int inLine;
    if (fread(&inLine, sizeof(int), 1, fp) != 1)
        return false;
    for(int i = 0 ; i < inLine ; i ++)
    {
        CLine t;
        t.LoadBinary(fp);
        vecSpecialLineList_.push_back(t);
    }

    if (fread(&onlyUseInRectFeature, sizeof(bool), 1, fp) != 1)
        return false;

    if (fread(&isUseUseMultiFrameLoc, sizeof(bool), 1, fp) != 1)
        return false;

    if (fread(&iMultiFrameLocDequeSize, sizeof(int), 1, fp) != 1)
        return false;

    if (fread(&fMaDisPowerfulValue, sizeof(float), 1, fp) != 1)
        return false;

    if (fread(&isUseSinglePointLoc, sizeof(bool), 1, fp) != 1)
        return false;
		
    if (fread(&m_MaxDisWithoutFeature, sizeof(float), 1, fp) != 1)
        return false;
	std::cout<<"load m_MaxDisWithoutFeature: "<<m_MaxDisWithoutFeature<<std::endl;
#endif

    return true;
}

bool CFeatureLocalizationParam::SaveBinary(FILE *fp)
{
    float f[2] = {minRange, maxRange};
    if (fwrite(f, sizeof(float), 2, fp) != 2)
        return false;

    if (fwrite(&nMostMatchCount_, sizeof(int), 1, fp) != 1)
        return false;

    f[0] = angleEqualLimit;
    f[1] = rangeEqualLimit;
    if (fwrite(f, sizeof(float), 2, fp) != 2)
        return false;

    int n[2] = {criteritonThreshold, refEfficientPointCount};
    if (fwrite(n, sizeof(int), 2, fp) != 2)
        return false;

    if (fwrite(&ratio, sizeof(float), 1, fp) != 1)
        return false;

    int inPoint = vecSpecialPntList_.size();
    if (fwrite(&inPoint, sizeof(int), 1, fp) != 1)
        return false;
    for(int i = 0 ; i < inPoint ; i ++)
    {
        vecSpecialPntList_.at(i).SaveBinary(fp);
    }

    int inLine = vecSpecialLineList_.size();
    if (fwrite(&inLine, sizeof(int), 1, fp) != 1)
        return false;
    for(int i = 0 ; i < inLine ; i ++)
    {
        vecSpecialLineList_.at(i).SaveBinary(fp);
    }

    if (fwrite(&onlyUseInRectFeature, sizeof(bool), 1, fp) != 1)
        return false;


    if (fwrite(&isUseUseMultiFrameLoc, sizeof(bool), 1, fp) != 1)
        return false;

    if (fwrite(&iMultiFrameLocDequeSize, sizeof(int), 1, fp) != 1)
        return false;

    if (fwrite(&fMaDisPowerfulValue, sizeof(float), 1, fp) != 1)
        return false;

    if (fwrite(&isUseSinglePointLoc, sizeof(bool), 1, fp) != 1)
        return false;

    std::cout<<"m_MaxDisWithoutFeature: "<<m_MaxDisWithoutFeature<<std::endl;
    if (fwrite(&m_MaxDisWithoutFeature, sizeof(float), 1, fp) != 1)
        return false;

    return true;
}
//   Operator "="
void CFeatureLocalizationParam::operator =(CFeatureLocalizationParam obj)
{
    ratio = obj.ratio;
    minRange =obj.minRange;
    maxRange = obj.maxRange;
    nLeastMatchCount_  = obj.nLeastMatchCount_;
    nMostMatchCount_ = obj.nMostMatchCount_;
    fSamePointMaxDist_ = obj.fSamePointMaxDist_;
    angleEqualLimit = obj.angleEqualLimit;
    rangeEqualLimit = obj.rangeEqualLimit;
    criteritonThreshold = obj.criteritonThreshold;
    refEfficientPointCount = obj.refEfficientPointCount;
    fLineEqualLimit_[0] = obj.fLineEqualLimit_[0];
    fLineEqualLimit_[1] = obj.fLineEqualLimit_[1];
    fLineEqualLimit_[2] = obj.fLineEqualLimit_[2];
    fLineEqualLimit_[3] = obj.fLineEqualLimit_[3];
    fLineEqualLimit_[4] = obj.fLineEqualLimit_[4];
    fMinLineLen_ = obj.fMinLineLen_;
}

void CFeatureLocalizationParam::SetValue(const CFeatureLocalizationParam &obj)
{
    ratio = obj.ratio;
    minRange =obj.minRange;
    maxRange = obj.maxRange;
    nLeastMatchCount_  = obj.nLeastMatchCount_;
    nMostMatchCount_ = obj.nMostMatchCount_;
    fSamePointMaxDist_ = obj.fSamePointMaxDist_;
    angleEqualLimit = obj.angleEqualLimit;
    rangeEqualLimit = obj.rangeEqualLimit;
    criteritonThreshold = obj.criteritonThreshold;
    refEfficientPointCount = obj.refEfficientPointCount;
    fLineEqualLimit_[0] = obj.fLineEqualLimit_[0];
    fLineEqualLimit_[1] = obj.fLineEqualLimit_[1];
    fLineEqualLimit_[2] = obj.fLineEqualLimit_[2];
    fLineEqualLimit_[3] = obj.fLineEqualLimit_[3];
    fLineEqualLimit_[4] = obj.fLineEqualLimit_[4];
    fMinLineLen_ = obj.fMinLineLen_;
}
///////////////////////////////////////////////////////////////////////////////
//   特征定位应用矩形: 定义在该矩形区域内的特征定位参数。

//
//   从二进制文件读取数据。
//
bool CFeatureLocalizationRect::LoadBinary(FILE *fp)
{
    if (!CRectangle::LoadBinary(fp))
        return false;

    return param_.LoadBinary(fp);
}

//
//   将数据写入二进制文件。
//
bool CFeatureLocalizationRect::SaveBinary(FILE *fp)
{
    if (!CRectangle::SaveBinary(fp))
        return false;

    return param_.SaveBinary(fp);
}
