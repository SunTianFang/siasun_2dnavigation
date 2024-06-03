#ifndef __CRectFeature
#define __CRectFeature

#include "Feature.h"
#include "LineFeature.h"
#include "CornerPointFeature.h"
#include "PointFeature.h"
#include "Geometry.h"

class CRectFeature : public CFeature
{
public:
	CRectangle m_Rect;
	float m_fx, m_fy;//中心位置
	float m_fWidth, m_fLength;//矩形size
	vector<CLineFeature> m_vLines;//直线
	vector<CCornerPointFeature> m_vCorner;//角点
        vector<CScanPoint> m_vPoints;

public:
	CRectFeature();
	CRectFeature(vector<CScanPoint> &vPoints);
	CRectFeature(CRectFeature *another);
	CRectFeature(CRectangle rect);

	void Creat(CRectangle rect);

        virtual bool LoadBinary(FILE* fp);

        virtual bool SaveBinary(FILE* fp);

        void Transform(Eigen::Affine3d tr);

        bool withinRange(float minX, float maxX, float minY, float maxY);

	//static bool combineFeatures(vector<CFeature*> vpF, CFlatReflectorFeature &frf, int radius);

#ifdef _MSC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF clr);
#endif
};
#endif
