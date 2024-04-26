#include "FeatureScan.h"
#include "blackboxhelper.hpp"

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

CFeatureScan::CFeatureScan()
{
    m_pLineFeatures = NULL;
    m_bIsOnlyRelector = false;
    m_iOnlyRelectorSize = 0 ;
}

CFeatureScan::~CFeatureScan()
{
    this->Clear();
    m_bIsOnlyRelector = false;
    m_iOnlyRelectorSize = 0 ;
}

CFeatureScan::CFeatureScan(int nNum):CScan(nNum)
{
    m_pLineFeatures = NULL;
    m_bIsOnlyRelector = false;
    m_iOnlyRelectorSize = 0 ;
}

void CFeatureScan::operator=(const CFeatureScan &Scan)
{
    *(GetScanPointCloudPointer()) = *(((CScan&)Scan).GetScanPointCloudPointer());

    m_poseScanner = Scan.m_poseScanner;
    m_fStartAng = Scan.m_fStartAng;
    m_fEndAng = Scan.m_fEndAng;

    if (m_pLineFeatures != NULL)
    {
        delete m_pLineFeatures;
        m_pLineFeatures = NULL;
    }
    if (Scan.m_pLineFeatures != NULL)
        m_pLineFeatures = Scan.m_pLineFeatures->Duplicate();

    for(deque<CPnt>::const_iterator it = Scan.m_ptReflectors.begin(); it != Scan.m_ptReflectors.end(); ++it)
    {
        m_ptReflectors.push_back(*it);
    }

    m_bIsOnlyRelector = Scan.m_bIsOnlyRelector;
    m_iOnlyRelectorSize = Scan.m_iOnlyRelectorSize;

}

void CFeatureScan::Clear()
{
    CScan::Clear();
    if (m_pLineFeatures != NULL)
    {
        delete m_pLineFeatures;
        m_pLineFeatures = NULL;
    }
    m_ptCorners.clear();
    m_ptEdges.clear();
    m_CornerPairs.clear();
    m_ptReflectors.clear();
    m_ptLineEdges.clear();

    m_bIsOnlyRelector = false;
    m_iOnlyRelectorSize = 0 ;
}

bool CFeatureScan::CreateLineFeatures(CLineFeatureCreationParam *pParam)
{
    if (m_pLineFeatures != NULL)
    {
        delete m_pLineFeatures;
        m_pLineFeatures = NULL;
    }
    m_pLineFeatures = new CLineFeatureSet(*this);
    return (m_pLineFeatures != NULL);
}
//
//   生成所有点特征。
//   注意：此函数必须在已生成过所有直线特征之后(即调用过CreateLineFeatures()之后)调用！
//
bool CFeatureScan::CreatePointFeatures()
{
    int LeadEdge = 0;
    int PostEdge = 0;
    int Num = 0;
    m_ptReflectors.clear();
    for(int i = 0; i < (m_nCount-1); i++)
    {
        int Criteriton = 254/*FeatureParam->criteritonThreshold*/; //wt_test_intensity 20230221
        if(m_pPoints[i].m_nIntensity < Criteriton && m_pPoints[i + 1].m_nIntensity >= Criteriton)
            LeadEdge = i;
        if(m_pPoints[i].m_nIntensity >= Criteriton && m_pPoints[i + 1].m_nIntensity < Criteriton)
        {
            PostEdge = i;
            if(LeadEdge != 0 && PostEdge != 0 && PostEdge > LeadEdge)
            {
                Num = PostEdge - LeadEdge;
                if(Num > FeatureParam->refEfficientPointCount)
                {
                    int p = LeadEdge + 1 + Num / 2;
                    m_pReflectors.x = m_pPoints[p].x;
                    m_pReflectors.y = m_pPoints[p].y;
                    // 构造扫描线
                    CLine ln(m_poseScanner.GetPntObject(), m_pReflectors);
                    // 计算扫描角
                    CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
                    m_pReflectors.r = ln.Length();
                    m_pReflectors.a = CAngle::NormAngle2(ang.m_fRad);
                    float distance = 1.0f;
                    if(!m_ptReflectors.empty())
                     distance= m_pReflectors.DistanceTo(m_ptReflectors.back());
                     //merge point 如果识别出的反光板之间的距离小于8ｃｍ说明他们是同一个反光板
                    if(!m_ptReflectors.empty() && distance < 0.08f)
                    {
                        m_ptReflectors.back().r = (m_ptReflectors.back().r + m_pReflectors.r) / 2;
                        m_ptReflectors.back().a = (CAngle::NormAngle2(m_ptReflectors.back().a) +
                                CAngle::NormAngle2(m_pReflectors.a)) / 2;
                        continue;
                    }
                    else if(!m_ptReflectors.empty() && distance >= 0.08f && distance < 0.5f)
                    {
                        m_ptReflectors.pop_back(); //删除上一个
                        continue;
                    }
                    if(m_pReflectors.r < FeatureParam->maxRange && m_pReflectors.r > FeatureParam->minRange)
                    {
                        m_ptReflectors.push_back(m_pReflectors);
                    }
                }
                LeadEdge = 0;
            }
        }
    }
    return true;
}


bool CFeatureScan::CreatePointFeaturesNew()
{
    int LeadEdge = 0;
    int PostEdge = 0;
    int Num = 0;
    m_ptReflectors.clear();
    m_ptHighBrightPnt.clear();
    for(int i = 0; i < (m_nCount - 1); i++)
    {
        int Criteriton = 254/*FeatureParam->criteritonThreshold*/; //wt_test_intensity 20230116
        CPnt PntTemp;
        stReflector ostReflector;

        if(m_pPoints[i].m_nIntensity >= Criteriton)
        {
            PntTemp.x = m_pPoints[i].x;
            PntTemp.y = m_pPoints[i].y;
            // 构造扫描线
            CLine ln(m_poseScanner.GetPntObject(), PntTemp);
            // 计算扫描角
            CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
            PntTemp.r = ln.Length();
            PntTemp.a = CAngle::NormAngle2(ang.m_fRad);
            ostReflector.m_ptLocal = PntTemp;
            m_ptHighBrightPnt.push_back(ostReflector);
        }

        if(m_pPoints[i].m_nIntensity < Criteriton && m_pPoints[i + 1].m_nIntensity >= Criteriton)
            LeadEdge = i;
        if(m_pPoints[i].m_nIntensity >= Criteriton && m_pPoints[i + 1].m_nIntensity < Criteriton)
        {
            PostEdge = i;
            if(LeadEdge != 0 && PostEdge != 0 && PostEdge > LeadEdge)
            {
                Num = PostEdge - LeadEdge;
                if(Num >= FeatureParam->refEfficientPointCount)
                {
                    float x_sum = 0.0; //平面直角坐标X位置
                    float y_sum = 0.0; //平面直角坐标Y位置

                    for(int m = (LeadEdge + 1); m < (PostEdge + 1); m++) //wt_fix 20220905 修正高亮点的选取
                    {
                        x_sum += m_pPoints[m].x;
                        y_sum += m_pPoints[m].y;
                    }
                    m_pReflectors.x = x_sum / Num;
                    m_pReflectors.y = y_sum / Num;

                    //构造扫描线
                    CLine ln(m_poseScanner.GetPntObject(), m_pReflectors);
                    //计算扫描角
                    CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
                    m_pReflectors.r = ln.Length();
                    m_pReflectors.a = CAngle::NormAngle2(ang.m_fRad);
                    float distance = 1.0f;
                    if(!m_ptReflectors.empty())
                        distance = m_pReflectors.DistanceTo(m_ptReflectors.back());
                     //merge point 如果识别出的反光板之间的距离小于8ｃｍ说明他们是同一个反光板
                    if(!m_ptReflectors.empty() && distance < 0.08f)
                    {
                        m_ptReflectors.back().x = (m_ptReflectors.back().x + m_pReflectors.x) / 2;
                        m_ptReflectors.back().y = (m_ptReflectors.back().y + m_pReflectors.y) / 2;

                        //构造扫描线
                        CLine ln(m_poseScanner.GetPntObject(), m_ptReflectors.back());
                        //计算扫描角
                        CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
                        m_ptReflectors.back().r = ln.Length();
                        m_ptReflectors.back().a = CAngle::NormAngle2(ang.m_fRad);
                        continue;
                    }
                    else if(!m_ptReflectors.empty() && distance >= 0.08f && distance < 0.5f)
                    {
                        m_ptReflectors.pop_back(); //删除上一个
                        continue;
                    }
                    if(m_pReflectors.r < FeatureParam->maxRange && m_pReflectors.r > FeatureParam->minRange)
                    {
                        m_ptReflectors.push_back(m_pReflectors);
                    }
                }
                LeadEdge = 0;
            }
        }
    }
    cout << "m_ptReflectors.size: " << m_ptReflectors.size() << endl;
    return true;
}


bool CFeatureScan::CreateLegFeatures()
{
    int LeadEdge = 0;
    int PostEdge = 0;
    int Num = 0;
    m_ptReflectors.clear();
    m_ptHighBrightPnt.clear();
    std::cout << "By Sam: 4" << std::endl;
    std::cout << "By Sam: m_nCount = " << m_nCount << std::endl;
    for(int i = 0; i < (m_nCount-1); i++)
    {
//        int Criteriton = FeatureParam->criteritonThreshold;
        int Criteriton = 254;

        if(m_pPoints[i].m_nIntensity < Criteriton && m_pPoints[i + 1].m_nIntensity >= Criteriton)
            LeadEdge = i;
        if(m_pPoints[i].m_nIntensity >= Criteriton && m_pPoints[i + 1].m_nIntensity < Criteriton)
        {
            PostEdge = i;
            if(LeadEdge != 0 && PostEdge != 0 && PostEdge > LeadEdge)
            {
                Num = PostEdge - LeadEdge;
                if(Num > FeatureParam->refEfficientPointCount)
                {
                    float x_sum = 0.0;            // 平面直角坐标X位置
                    float y_sum = 0.0;            // 平面直角坐标Y位置
                    for(int m = LeadEdge ; m < (PostEdge+1) ; m++ )
                    {
                        x_sum+=m_pPoints[m].x;
                        y_sum+=m_pPoints[m].y;
                    }
                    m_pReflectors.x = x_sum/(Num+1);
                    m_pReflectors.y = y_sum/(Num+1);

//                    std::cout << "By Sam: pReflectors = " << m_pReflectors.x << ", " << m_pReflectors.y << std::endl;
                    // 构造扫描线
                    CLine ln(m_poseScanner.GetPntObject(), m_pReflectors);
                    // 计算扫描角
                    CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
                    m_pReflectors.r = ln.Length();
                    m_pReflectors.a = CAngle::NormAngle2(ang.m_fRad);
                    float distance = 1.0f;
                    if(!m_ptReflectors.empty())
                     distance= m_pReflectors.DistanceTo(m_ptReflectors.back());
                     //merge point 如果识别出的反光板之间的距离小于8ｃｍ说明他们是同一个反光板
                    if(!m_ptReflectors.empty() && distance < 0.08f)
                    {
//                        m_ptReflectors.back().r = (m_ptReflectors.back().r + m_pReflectors.r)/2;
//                        m_ptReflectors.back().a = (CAngle::NormAngle2(m_ptReflectors.back().a) +
//                                CAngle::NormAngle2(m_pReflectors.a))/2;
                        m_ptReflectors.back().x = (m_ptReflectors.back().x + m_pReflectors.x) / 2;
                        m_ptReflectors.back().y = (m_ptReflectors.back().y + m_pReflectors.y) / 2;

                        //构造扫描线
                        CLine ln(m_poseScanner.GetPntObject(), m_ptReflectors.back());
                        //计算扫描角
                        CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
                        m_ptReflectors.back().r = ln.Length();
                        m_ptReflectors.back().a = CAngle::NormAngle2(ang.m_fRad);
                        continue;
                    }

                    if(m_pReflectors.r < FeatureParam->maxRange && m_pReflectors.r > FeatureParam->minRange)
                    {
                        m_ptReflectors.push_back(m_pReflectors);
                    }
                }
                LeadEdge = 0;
            }
        }
    }
    return true;
}


bool CFeatureScan::FindHighBrightPnt()
{
    m_ptHighBrightPnt.clear();

    for(int i = 0; i < m_nCount; i++)
    {
        int Criteriton = 254/*FeatureParam->criteritonThreshold*/; //wt_test 20230116
//        cout << "critertion" << i << ": " << m_pPoints[i].m_nIntensity << endl;
        CPnt PntTemp;
        stReflector ostReflector;
        if(m_pPoints[i].m_nIntensity >= Criteriton)
        {
            PntTemp.x = m_pPoints[i].x;
            PntTemp.y = m_pPoints[i].y;
            // 构造扫描线
            CLine ln(m_poseScanner.GetPntObject(), PntTemp);
            // 计算扫描角
            CAngle ang = ln.SlantAngle() - m_poseScanner.GetAngle();
            PntTemp.r = ln.Length();
            PntTemp.a = CAngle::NormAngle2(ang.m_fRad);
            ostReflector.m_ptLocal = PntTemp;
            m_ptHighBrightPnt.push_back(ostReflector);
        }
    }

    std::cout << "m_nCount: " << m_nCount << ", m_ptHighBrightPnt.size: " << m_ptHighBrightPnt.size() << std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "HighBrightPnt.size: ", m_ptHighBrightPnt.size());
#endif
    return true;
}
