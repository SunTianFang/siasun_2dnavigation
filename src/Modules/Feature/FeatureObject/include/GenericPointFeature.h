#ifndef __CGenericPointFeature
#define __CGenericPointFeature

//#include "Eigen/Eigen"
#include "Feature.h"
#include "scan/ScanPoint.h"
class CGenericPointFeature
{
private:
public:
        float m_fx;
        float m_fy;
        PointCluster m_vPoints;
        int m_nType;
public:
	static int POINT_COUNT_MIN;
	static int POINT_COUNT_MAX;
	static float POINT_DISTANCE_MIN;
	static float POINT_DISTANCE_MAX;
public:
	CGenericPointFeature();
        CGenericPointFeature(PointCluster& , int m_nType = 0);
	CGenericPointFeature(CGenericPointFeature *another);

        void clear();

        bool extract();

//        void Transform(Eigen::Affine3d tr);

        bool withinRange(float minX, float maxX, float minY, float maxY);

#ifdef _MFC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF clr);
#endif
};
#endif
