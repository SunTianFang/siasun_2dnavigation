#ifndef __CPosFinder
#define __CPosFinder

#include "Geometry.h"
#include "MatchTabSet.h"
#include "ScanMatcher.h"
#include "LeastSquareMethod.h"
#include "LineMatchList.h"
#include "FeatureMap.h"
#include "FeatureScan.h"
#include "FeatureMatchInfo.h"
#include<Eigen/Dense>
using namespace Eigen;
#define FULL_MAP_MODE                             0
#define QUICK_MAP_MODE                            1
///////////////////////////////////////////////////////////////////////////////
struct CSpecialRatio
{
    float  m_fXmin;
    float  m_fXmax;
    float  m_fYmax;
    float  m_fYmin;
    float  m_fRatio;
};

class CFeatureLocalization : public CScanMatcher
{
private:
    CTransform m_trans;
    vector <CSpecialRatio> SpecialRatioSet;
    float  m_fRatio;
private:
    int    m_nWorkMode;                 // 工作模式: 0 - 全局映射; 1 - 快速映射
    CFeatureLocalizationParam* m_Param;        // 参数
    CRectangle          *localizationRect_; //定位的矩形区域
    CPointFeatureSet* m_pWorldLayer;
    CPointFeatureSet m_RefLayer;                  // 当前的(世界坐标)参考层
    CPointFeatureSet m_LocalLayer;
    CLineFeatureSet* m_pWorldLines;     // 指向全局直线特征集合的指针
    CLineFeatureSet* m_pRefWorldLines;     // 指向全局直线特征集合的指针
    ULONG           m_ulLastVelUpdate;  // 上次更新速度的时间
    bool            m_bVelIsNew;        // 是否已获得激光头当前速度
    CPointMatchList     m_PointMatchList;
    CLineMatchList      m_LineMatchList;
    CLineMatchList      m_LineMatchListX;
    CLineMatchList      m_LineMatchListY;
    CFeatureMatchTabSet m_MatchTabSet;
    CLeastSquareMethod  m_LeastSquareMethod;
    float               m_fScore;        // 匹配得分(0~1)
    CPosture            m_pstLastGoodMap;          // 上一次成功的定位得到的姿态
    int                 m_nFullMapCount;
    BOOL                m_bLocalFullMap; //局部全局匹配标志
    vector<CLine>       vecLines_;      //识别到的直线

    int m_iLegFalseNum;
    CPosture m_lastLegsCenter;
    int m_iLegTimes;
    int m_iLegPoseCount;
    deque<CPosture> m_RecentPoses;
    int m_iFaileTimes;

    vector<CPointFeatureSet> m_ScatterPointFeatureSet;
    vector<MatrixXd>  m_cov;
    vector <stVariousDis> m_vecVariousDis;
    deque <deque<stReflector> > m_DequeReflectors;//记录着三帧数据
private:
    // 在当前层内进行快速配准
    bool PointFeatureQuickMatch();
    bool PointFeatureQuickMatchUseMaDis();
    bool PointFeatureNewQuickMatchUseMaDis();
    bool PointFeatureQuickMatchUseDequeReflectors(CScan *pCurScan);
    bool PointFeatureQuickMatchUseDequeReflectorsUseInsideMaDis(CScan *pCurScan);


    bool PointFeatureFullMapMatch(float equal_limit);
    bool PointFeatureFindLegPairs(float equal_limit);  // By Sam Add

    short DirectRegister(CTransform trans, CPointMatchList* tab);

    // 在点云中找到所有的“点状特征”
    bool CreatLocalLayerFromCurScan(CScan* pScan);

    bool AddPntToLocalLayer(CPnt pntLocal);

    // 在点云中找到所有的“直线特征”
    bool MatchLineFeatures(CScan* pCurScan);

    // 主定位流程
    int LocalizationDistribution(CPosture* pResultPosture);

    int LocalizationByLeg(CPosture* pResultPosture);   // By Sam Add For LegMatch

    bool FilterLegPose(CPosture& pose);   // By Sam Add For LegMatch

    int LocalizationWithCorridor(CPosture* pResultPosture, CPosture &estimatePost);

    int LocalizationWithSinglePoint(CPosture* pResultPosture, CPosture &estimatePost);

    int LocalizationWithDoublePoint(CPosture* pResultPosture, CPosture &estimatePost);

    int FullMapLocalization(CPosture* pResultPosture);

    // 在指定的层内进行快速影射定位
    int QuickMapLocalization(CPosture* pResultPosture);

    void AddLinePair();

    void SwitchRatio(const CPosture&);

    void ClearMatchList();

public:
    CFeatureLocalization();

    void CopyMatchPair(CFeatureMatchInfo  &MatchInfo);

    void CopyIdentifierFeature(CFeatureMatchInfo  &MatchInfo);

    // 设置特征图
    bool SetFeatureMap(CFeatureMap* pFeatureMap);

    // 设置参数
    virtual bool SetScannerParam(float fStartAng, float fEndAng, float fAngReso, float fMaxRange);

    // 设置初始姿态
    virtual void SetInitPosture(const CPosture& pst, bool bRough = false);

    // 根据里程计设置姿态
    virtual void SetOdometricPosture(const CPosture& pst, const int);

    // 设置定位参数
    void SetLocalizationParam(CFeatureLocalizationParam* Param) { m_Param = Param; }
    void SetLocalizationRect(CRectangle * r) { localizationRect_ = r; }

    // 提供匹配功能(成功时返回1，失败时返回负值)
    virtual int LocalizationPro(CScan* pCurScan, CPosture& pstNew, CPosture &estimatePost);

    virtual int FeatureLocalizationUseSingleframe(CScan* pCurScan, CPosture& pstNew, CPosture &pstEstimate);

    virtual int FeatureLocalizationUseMultiframe(CScan* pCurScan, CPosture& pstNew, CPosture &pstEstimate);


    // 设置经匹配运算校正后的姿态
    void SetCorrectedPosture(const CPosture& pst);

    // 取得“世界层”
    CPointFeatureSet& GetWorldLayer() { return *m_pWorldLayer; }

    // 取得“参考层”
    CPointFeatureSet& GetReferenceLayer() { return m_RefLayer; }

    // 取得“本地层”
    CPointFeatureSet& GetLocalLayer() { return m_LocalLayer; }

    int GetFeaturePairCount()
    {
        int sumcount = m_PointMatchList.GetCount() + m_LineMatchList.GetCount();
        return sumcount;
    }


    void SetLocalFullMap(BOOL b)
    {
        m_bLocalFullMap = b;
    }

    float GetMatchRatio()
    {
        return m_fScore;
    }

    virtual int GetMatchCount()
    {
        return GetFeaturePairCount();
    }

    long GetRefWorldLinesNum();

    void EvaluateTransform(const CTransform&);

    bool ReSetLegPose();

};
#endif
