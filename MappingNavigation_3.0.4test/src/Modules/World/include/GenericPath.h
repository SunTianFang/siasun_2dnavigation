#pragma once

#include "Path.h"
#include "ArcPath.h"
#include "Bezier.h"

//////////////////////////////////////////////////////////////////////////////
//   定义"CGenericPath"类: 该类从"CPath"继承而来，用来定义一般的曲线路径。
//
class DllExport CGenericPath : public CPath
{
protected:
	CAngle   m_angStartHeading;     // 车体在起始节点处的方向角
	CAngle   m_angEndHeading;       // 车体在终止节点处的方向角
	int      m_nCountCtrlPoints;    // 曲线中控制点的数量(不包括两端节点)
	CPnt* m_pptCtrl;            // 指向曲线中各个控制点的指针(不包括两端节点)
	

	CPosture m_pstStart;            // 起始姿态
	CPosture m_pstEnd;              // 终止姿态

	bool	 m_bTangency;		 //运行方式为相切还是平移, 运动方式 0:相切 1:平移

public:
	CBezier  m_Curve;               // 曲线对象
											  // 生成任意阶贝塞尔曲线
private:
	// 生成任意阶贝塞尔曲线路径
	bool Create(CPosture& pstStart, CPosture& pstEnd, int nCountCtrlPoints, CPnt* pptCtrl);

	// 生成三阶贝塞尔曲线路径
	bool Create(CPosture& pstStart, CPosture& pstEnd, float fLen1, float fLen2);

protected:
	bool Init();

public:
	// 构造函数: 生成三阶贝塞尔曲线
	CGenericPath(int uId, int nStartNode, int nEndNode, CPosture& pstStart, 
		CPosture& pstEnd, float fLen1, float fLen2, float fVeloLimit = 0.6f, 
		SHORT nGuideType = TAPE_GUIDANCE, USHORT uObstacle = 0, 
		USHORT uDir = POSITIVE_HEADING, USHORT uExtType = 0);

	// 缺省构造函数
	CGenericPath();

	// 析构函数
	~CGenericPath();

	// 根据圆弧路段生成Bezier曲线路段
	bool CreateFromArcPath(CArcPath& ArcPath);

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd);

	virtual bool IsTangency() {return m_bTangency;}

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	// Fuzzy node checking limit (5 degree)
	virtual float FuzzyNodeCheckLimit() { return 0.087f; }

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

	// 从二进制文件生成对象数据
	virtual bool Create(CArchive& ar);

	// 将对象数据写入到二进制文件中
	virtual bool Save(CArchive& ar);

#ifdef _MFC_VER
	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 2);
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);

	// 画出此路径曲线的控制点
	void DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr);

#elif defined QT_VERSION
	virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef) {return 0;}
	virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth = 2) {}

	// 画出此路径曲线的控制点
	void DrawCtrlPoints(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr) {}
#endif
};
