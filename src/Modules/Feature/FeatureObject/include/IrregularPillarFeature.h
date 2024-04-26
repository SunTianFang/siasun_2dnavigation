#ifndef __CIrregularPillarFeature
#define __CIrregularPillarFeature

#include "PointFeature.h"

#define SHORT_LINE_MAX_DIST_CHANGE            50      // 最大允许距离变化量
#define MAX_POINT_NUM_IN_SHORT_LINE           500       // 不规则圆柱中含点的最大数量 25
#define IRREGULAR_FEATURE_REG_DISTANCE_WINDOW 300

class CScanPointCloud;

///////////////////////////////////////////////////////////////////////////////
//   不规则圆柱特征(视为一种点特征)。
class CIrregularPillarFeature : public CPointFeature
{
protected:
	float m_fWidth;                // 不规则圆柱段的宽度
	float m_fAngle;                // 不规则圆柱段在世界坐标内角度
        CLine m_ln;                    // 对应的直线段
	int   m_nWhichSideToUse;       // 使用它的哪一面(或双面)
	float m_fMaxIncidenceAngle;    // 每面最大可用入射角(0~PI/2)
	int   m_nMinNumPoints;         // 最少点数
	float m_fMaxLength;            // 线段最大长度

public:
	CIrregularPillarFeature();

	// 设置不规则圆柱特征的参数
	virtual void SetParam(float fWidth, float fAngle, int nWhichSideToUse, 
		float fMaxIncidenceAngle, int nMinNumPoints, float fMaxLength);

	// 判断不规则圆柱段是否被指定的扫描线照到，并返回扫描点坐标
        virtual bool HitByLineAt(CLine& lnRay, CPnt& ptHit, float& fDist);

	// 针对给定的点云，在规定的角度范围内，检测点云中是否含有该不规则圆柱段特征，并返回它的中心位置
        virtual bool Detect(CPosture& pst, CScan* pScan, float fStartAngle, float fEndAngle, CPnt* ptCenter);

	// 生成一个复本
	virtual CPointFeature* Duplicate();

	// 判断指定的入射光线是否可用
        virtual bool CheckInRay(CLine& lnRay);

	// 从文件中装入点特征的参数
	virtual bool LoadBinary(FILE* fp);

	// 将点特征的参数保存到文件中
	virtual bool SaveBinary(FILE* fp);

#ifdef _MSC_VER
	// 在屏幕上绘制此点特征
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nSize);
#endif
};
#endif

