#ifndef __CMultiSegLine
#define __CMultiSegLine

#include "Geometry.h"
#include "DataRange.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPoint>
#include <QPainter>
#endif

///////////////////////////////////////////////////////////////////////////////
//   定义"CMultiSegLine"类，用来表示共线的多段直线。
class CMultiSegLine : public CLine, public CDataRangeSet
{
public:
    CMultiSegLine(const CLine& line);
    CMultiSegLine(const CMultiSegLine& another);
    CMultiSegLine() {}

    // 重载"="操作符，将对象设置为仅含有一段直线段
    void operator =(const CLine &line);

	// 取得指定的子段
	bool GetSegment(int i, CLine& Line) const;

	// 判断一个给定的点是否“触碰”到该直线
	bool PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	// 在屏幕上绘制此直线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#elif defined QT_VERSION
    // 在屏幕上绘制此直线
    void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};
#endif
