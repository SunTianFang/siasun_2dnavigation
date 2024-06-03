#ifndef __CCornerPointFeature
#define __CCornerPointFeature

//#include "feature/FeatureSet.h"
#include <vector>
#include "LineFeature.h"
#include "PointFeature.h"
#include "ScanPoint.h"
#include "Eigen/Eigen"

using namespace std;

class CFeature;
class CLineFeature;

class CCornerPointFeature : public CPointFeature
{
  public:
    float m_fx, m_fy;
    int m_nCount;
    vector<CScanPoint> m_vPoints;

  public:
    static int POINT_COUNT_MIN;
    static int POINT_COUNT_MAX;
    static float POINT_DISTANCE_MIN;
    static float POINT_DISTANCE_MAX;

  public:
    CCornerPointFeature();
    CCornerPointFeature(vector<CScanPoint> &vPoints);
    CCornerPointFeature(CCornerPointFeature *another);
    CCornerPointFeature(const float &x, const float &y);

    bool ExtractFromLineSet(std::vector<CLineFeature> &v);

    bool extract();
    virtual bool LoadBinary(FILE *fp);
    virtual bool SaveBinary(FILE *fp);

    void Transform(Eigen::Affine3d tr);

    bool withinRange(float minX, float maxX, float minY, float maxY);

#ifdef _MFC_VER
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clr);
#endif
};
#endif
