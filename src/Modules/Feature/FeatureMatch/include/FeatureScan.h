#ifndef CFEATURESCAN_H
#define CFEATURESCAN_H
#include "Scan.h"
#include "LineFeatureSet.h"
#include <deque>
#include "FeatureLocArea.h"
class CEdgePoint : public CPnt
{
public:
        int m_nEdgeType;

public:
        CEdgePoint()
        {
                m_nEdgeType = -1;
        }
};

class CCornerPointsPair
{
public:
        int nIndex1;
        int nIndex2;

public:
        CCornerPointsPair(int _nIndex1, int _nIndex2)
        {
                nIndex1 = _nIndex1;
                nIndex2 = _nIndex2;
        }

        bool operator == (const CCornerPointsPair& another)
        {
                if ((nIndex1 == another.nIndex1 && nIndex2 == another.nIndex2) ||
                         (nIndex1 == another.nIndex2 && nIndex2 == another.nIndex1))
                        return true;
                else
                        return false;
        }
};

class stReflector
{
    public:
    stReflector(){m_bIsMatched = false;}
    CPnt m_ptLocal;                // 反光点在局部坐标系中的坐标
    CPnt m_ptWorld;               // 反光点在世界坐标系中的坐标
    CPnt m_ptMatchMapWorld;      // 匹配到的地图中反光板坐标
    bool m_bIsMatched;          //是否已经匹配成功
    // Copy constructor
    void operator=(const stReflector &ostReflector)
    {
         m_ptLocal = ostReflector.m_ptLocal;
         m_ptWorld = ostReflector.m_ptWorld;
         m_ptMatchMapWorld = ostReflector.m_ptMatchMapWorld;
         m_bIsMatched = ostReflector.m_bIsMatched;
    }
};

class CFeatureScan: public CScan
{
public:
    CPosture  m_poseScanner;
    CPosture  m_pstRobot;                // 估测得到的机器人的姿态
    CPosture  sc_ScannerPos;

    CLineFeatureSet* m_pLineFeatures;
    deque<CPnt> m_ptCorners;
    deque<CEdgePoint> m_ptEdges;
    deque<CCornerPointsPair> m_CornerPairs;
    deque<CPnt> m_ptReflectors;
    CPnt m_pReflectors;
    vector<CPnt> m_ptLineEdges;
    CFeatureLocalizationParam*    FeatureParam;
    vector<stReflector> m_ptHighBrightPnt;//高亮点

    bool   m_bIsOnlyRelector;
    int    m_iOnlyRelectorSize;
public:
    CFeatureScan();
    ~CFeatureScan();
    // 构造具有nNum个点的CScan对象
    CFeatureScan(int nNum);
    // 重载“=”操作符
    void operator=(const CFeatureScan &Scan);
    // 清除所有数据
    void Clear();
    //生成线特征
    bool CreateLineFeatures(CLineFeatureCreationParam* pParam = NULL);
    //生成所有点特征
    bool CreatePointFeatures();//by intensity
    bool CreatePointFeaturesNew();//by intensity
    bool CreateLegFeatures();//by intensity Sam Add
    //分辨出高亮的反光点
    bool FindHighBrightPnt();
    //设置定位参数
    void SetFeatureParam(CFeatureLocalizationParam* Param){FeatureParam = Param;}
};

#endif // CFEATURESCAN_H
