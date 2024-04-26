#include "stdafx.h"
#include "FeatureMethod.h"
#include "AffinePosture.h"

extern bool readJffFormat;

///////////////////////////////////////////////////////////////////////////////
//   实现基于特征的定位方法。

CFeatureMethod::CFeatureMethod()
{
    param_  = NULL;
    type_ = 1;
    map_ = CreateMap();
    if (map_ != NULL)
        map_->Init();
}

CFeatureMethod::~CFeatureMethod()
{
    if (map_ != NULL)
        delete map_;
    map_ = NULL;

}

CFeatureMap *CFeatureMethod::CreateMap()
{
    return new CFeatureMap;
}

bool CFeatureMethod::Initialize()
{
    return true;
}
bool CFeatureMethod::UnloadMap()
{
    //dq 9.2
    /*
    if (map_ != NULL)
    {
        delete map_;
        map_ = NULL;
    }
    */
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CFeatureMethod::CreateLocParam()
{
    return new CFeatureLocalizationParam;
}

//
//   应用指定的定位参数。
//
bool CFeatureMethod::ApplyParam(const CLocalizationParam *p)
{
    param_ = ((CFeatureLocalizationParam*)p);
    FeatureLocalization_.SetLocalizationParam(param_);
    return true;
}
CFeatureScan *CFeatureMethod::CreatScanFromPointCloud(const ndt_oru::CStampedPointCloud &cloudIn,const bool bIsOnlyRefletor)
{
    if(!bIsOnlyRefletor)
    {
        // 为扫描结果数据分配存储空间
        CFeatureScan *pScan = new CFeatureScan(int(cloudIn.size()));
        // 构建Scan
        cloudIn.ToScan(pScan);
        // 均为以车体为中心的坐标系
        ((CFeatureScan *)pScan)->m_poseScanner.SetPosture(0.0, 0.0, 0.0);
        ((CFeatureScan *)pScan)->SetFeatureParam(param_);

        return pScan;
    }
    else
    {
        // 为扫描结果数据分配存储空间
        CFeatureScan *pScan = new CFeatureScan(int(cloudIn.size()));
        // 构建Scan
        int iSize = cloudIn.ToScanOnlyHighIntensity(pScan);
        // 均为以车体为中心的坐标系
        ((CFeatureScan *)pScan)->m_poseScanner.SetPosture(0.0, 0.0, 0.0);
        ((CFeatureScan *)pScan)->SetFeatureParam(param_);
        pScan->m_bIsOnlyRelector = true;
        pScan->m_iOnlyRelectorSize = iSize;
        return pScan;
    }
}

//
//   对应于该定位方法的定位流程。
//
bool CFeatureMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                                  const ndt_oru::CStampedPointCloud cloudIn,
                                  Eigen::Affine3d &estimatePose)
{

    FeatureLocalization_.SetLocalizationRect(this->localizationRect_);
    CPosture InitPosture = AffineToPosture(initPose);
    CPosture EstimatePosture = AffineToPosture(estimatePose);
    if (localMode == QUICK_MAP_MODE)
    {
        //快速匹配
        FeatureLocalization_.SetOdometricPosture(InitPosture, QUICK_MAP_MODE);
    }
    else if (localMode == FULL_MAP_MODE)
    {
        //全局匹配
        FeatureLocalization_.SetOdometricPosture(InitPosture, FULL_MAP_MODE);
        FeatureLocalization_.SetLocalFullMap(true);    //全局定位中的局部定位
    }
    CFeatureScan *pScan = CreatScanFromPointCloud(cloudIn);
    CPosture pstNew;
    bool bRet = false;

    cout << "initpose: " << InitPosture.x << ", " << InitPosture.y << ", " << InitPosture.fThita << endl;
    cout << "estimatepose: " << EstimatePosture.x << ", " << EstimatePosture.y << ", " << EstimatePosture.fThita << endl;

    //定位成功
    if (FeatureLocalization_.LocalizationPro(pScan, pstNew, EstimatePosture) > 0)
    {
        std::cout<<"-----------------------------FeatureLocalization_: success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        FeatureLocalization_.SetCorrectedPosture(pstNew);
        CPosture pst(pstNew.x , pstNew.y, pstNew.fThita);
        estimatePose = PostureToAffine(pst);
        localMode_ = localMode;
        pstNew_ = pst;
        initPose_ = initPose;
        bRet = true;
        matchInfo_.result_ = CMatchInfo::MATCH_OK;
        matchInfo_.pst_ = pstNew;
        matchInfo_.initPst_ = InitPosture;
        FeatureLocalization_.CopyMatchPair(matchInfo_);
        FeatureLocalization_.CopyIdentifierFeature(matchInfo_);
    }
    else    //定位失败
    {
        std::cout<<"-----------------------------FeatureLocalization_: failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        estimatePose = PostureToAffine(EstimatePosture);
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        matchInfo_.pst_ = InitPosture;
        FeatureLocalization_.CopyMatchPair(matchInfo_);
        FeatureLocalization_.CopyIdentifierFeature(matchInfo_);
    }
    if (pScan)
    {
        delete pScan;
        pScan = NULL;
    }
    return bRet;
}

//
//   取得匹配数据。
//
CMatchInfo *CFeatureMethod::GetMatchInfo()
{
    matchInfo_.matchRatio_ = FeatureLocalization_.GetMatchRatio();
    matchInfo_.matchNum_ = FeatureLocalization_.GetMatchCount();
    return &matchInfo_;
}


//
//   对定位质量进行评估。
//
bool CFeatureMethod::EvaluateQuality(float &score)
{
    float MatchingRate = FeatureLocalization_.GetMatchRatio();
    score = MatchingRate;
    CPosture m_CurOdmPos = AffineToPosture(initPose_);
    double dx = pstNew_.x - m_CurOdmPos.x;
    double dy = pstNew_.y - m_CurOdmPos.y;
    float fAngErr = NormAngle2(pstNew_.fThita - m_CurOdmPos.fThita);

    // 如果位置发生跳变，在此报错
    if( localMode_ == QUICK_MAP_MODE) //quick match
    {
        if(MatchingRate >= 90.0f &&(fabs(dx) > 0.5f || fabs(dy) > 0.5f|| fabs(fAngErr) > 0.2f))
        {
            return false;
        }
        else if (MatchingRate < 90.0f &&(fabs(dx) > 0.25f || fabs(dy) > 0.25f))
        {
            return false;
        }
    }
    else if(localMode_ == FULL_MAP_MODE) //full match
    {
        if(MatchingRate >= 90.0f &&(fabs(dx) > 1.0f || fabs(dy) > 1.0f|| fabs(fAngErr) > 0.2f))
        {
            return false;
        }
        else if (MatchingRate < 90.0f &&(fabs(dx) > 0.50f || fabs(dy) > 0.50f|| fabs(fAngErr) > 0.3f))
        {
            return false;
        }
        else if(MatchingRate < 80.0f)
            return false;
    }
    return true;
}

//
//   从二进制文件装入特征地图。
//
bool CFeatureMethod::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{
    if (!map_->LoadBinary(fp))
        return false;

    if (!readJffFormat)
    {
        int dummy;
        fread(&dummy, sizeof(int), 1, fp);
    }

    FeatureLocalization_.SetFeatureMap(map_);

    return true;
}

//
//   将特征地图写入二进制文件。
//
bool CFeatureMethod::SaveBinary(FILE *fp, string filename)
{
    if (!map_->SaveBinary(fp))
        return false;

    int dummy = 0;
    fwrite(&dummy, sizeof(int), 1, fp);

    return true;
}


bool CFeatureMethod::GetLocParam()
{
    return param_->isUseUseMultiFrameLoc;
}

bool CFeatureMethod::ReSetMethod()
{
    std::cout << "By Sam: Into ReSetFeature to ReSetLegLastPose !!!!!!!!" << std::endl;
    FeatureLocalization_.ReSetLegPose();
    return true;
}
