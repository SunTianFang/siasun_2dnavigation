#include "stdafx.h"
#include "Geometry.h"
#include "Frame.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   将点移动指定的距离。
//
void CPnt::Move(float dx, float dy)
{
    x += dx;
    y += dy;
}

//
//   将点绕原点进行旋转。
//
void CPnt::Rotate(float fAng, float fCx, float fCy)
{
    float fX = (float)((x - fCx) * cos(fAng) - (y - fCy) * sin(fAng) + fCx);
    float fY = (float)((x - fCx) * sin(fAng) + (y - fCy) * cos(fAng) + fCy);

    x = fX;
    y = fY;
}

//
//   将点绕指定的中心点进行旋转。
//
void CPnt::Rotate(float fAng, const CPnt &ptCenter)
{
    float fX = (float)((x - ptCenter.x) * cos(fAng) - (y - ptCenter.y) * sin(fAng) + ptCenter.x);
    float fY = (float)((x - ptCenter.x) * sin(fAng) + (y - ptCenter.y) * cos(fAng) + ptCenter.y);

    x = fX;
    y = fY;
}

//
//   重载操作符 "==".
//
bool CPnt::operator==(const CPnt &pt) const
{
    return (x == pt.x && y == pt.y);
}

//
//   重载操作符 "!=".
//
bool CPnt::operator!=(const CPnt &pt) const
{
    return (x != pt.x || y != pt.y);
}

//
//   重载 "+="
//
void CPnt::operator+=(const CPnt &pt)
{
    x += pt.x;
    y += pt.y;
}

//
//   重载 "+"
//
CPnt CPnt::operator+(const CPnt &pt) const
{
    CPnt ptNew(x, y);
    ptNew += pt;
    return ptNew;
}

//
//   重载 "-="。
//
void CPnt::operator-=(const CPnt &pt)
{
    x -= pt.x;
    y -= pt.y;
}

//
//   重载 "-"。
CPnt CPnt::operator-(const CPnt &pt) const
{
    CPnt ptNew(x, y);
    ptNew -= pt;

    return ptNew;
}

//
//   比较两个点的“大小”(依据“左-下”原则)。
//
bool CPnt::operator<(const CPnt &pt) const
{
    if (x < pt.x)
        return true;
    else if (x == pt.x)
        return (y < pt.y);
    else
        return false;
}

//
//   比较两个点的“大小”(依据“左-下”原则)。
//
bool CPnt::operator>(const CPnt &pt) const
{
    if (x > pt.x)
        return true;
    else if (x == pt.x)
        return (y > pt.y);
    else
        return false;
}

//
//   计算两个点之间的距离.
//
float CPnt::DistanceTo(const CPnt &pt) const
{
    float dx = x - pt.x;
    float dy = y - pt.y;
    return (float)sqrt(dx * dx + dy * dy);
}

//
//   计算两个点之间的距离的平方。
//
float CPnt::Distance2To(const CPnt &pt2) const
{
    float d_x = x - pt2.x;
    float d_y = y - pt2.y;

    return d_x * d_x + d_y * d_y;
}

//
//   判断两个点是否可以近似地认为是一个点(相距非常近)。
//
bool CPnt::IsEqualTo(const CPnt &pt2, float limit) const
{
    if (Distance2To(pt2) < (limit * limit))
        return true;
    else
        return false;
}

//
//   根据迪卡尔坐标计算出点的极坐标。
//
void CPnt::UpdatePolar()
{
    // 计算极径
    r = (float)sqrt(y * y + x * x);

    // 计算极角
    // 如果极径太小，说明点处于原点附近，无极角
    if (r < 1E-7)
        a = 0;
    else
    {
        float fAngle = (float)atan2(y, x);
        a = CAngle::NormAngle(fAngle);
    }
}

//
//   以给定的姿态作为坐标系的X轴，根据迪卡尔坐标计算出点的极坐标。
//
void CPnt::UpdatePolar(const CPosture &pstRefFrame)
{
    // 计算极径
    r = DistanceTo(pstRefFrame);

    // 计算极角
    // 如果极径太小，说明点处于原点附近，无极角
    if (r < 1E-7)
        a = 0;
    else
    {
        float fAngle = (float)atan2(y - pstRefFrame.y, x - pstRefFrame.x);
        fAngle -= pstRefFrame.fThita;
        a = CAngle::NormAngle(fAngle);
    }
}

//
//   根据极坐标计算出点的迪卡尔坐标。
//
void CPnt::UpdateCartisian()
{
    x = r * (float)cos(a);
    y = r * (float)sin(a);
}

//
//   计算该点到另一点的角度距离。
//
float CPnt::AngleDistanceTo(const CPnt &pt) const
{
    return (float)fabs(CAngle::NormAngle(a - pt.a));
}

//
//   从文件装入点数据。
//
bool CPnt::LoadText(FILE *fp)
{
    if (fscanf(fp, "%d\t%f\t%f\t%f\t%f\n", &id, &x, &y, &r, &a) != 5)
        return false;

    return true;
}

//
//   将点数据写入文件。
//
bool CPnt::SaveText(FILE *fp)
{
    fprintf(fp, "%d\t%f\t%f\t%f\t%f\n", id, x, y, r, a);
    return true;
}

//
//   从二进制文件装入点数据。
//
bool CPnt::LoadBinary(FILE *fp)
{
    unsigned int i;
    float f[4];

    if (fread(&i, sizeof(int), 1, fp) != 1)
        return false;

    if (fread(f, sizeof(float), 4, fp) != 4)
        return false;

    id = i;
    x = f[0];
    y = f[1];
    r = f[2];
    a = f[3];

    return true;
}

//
//   将点数据写入二进制文件。
//
bool CPnt::SaveBinary(FILE *fp)
{
    if (fwrite(&id, sizeof(int), 1, fp) != 1)
        return false;

    float f[4] = {x, y, r, a};
    if (fwrite(f, sizeof(float), 4, fp) != 4)
        return false;

    return true;
}

//
//   进行坐标正变换。
//
void CPnt::Transform(const CFrame &frame)
{
    float c = (float)cos(frame.fThita);
    float s = (float)sin(frame.fThita);
    float dx = x - frame.x;
    float dy = y - frame.y;

    x = dx * c + dy * s;
    y = dy * c - dx * s;
}

//
//   进行坐标逆变换。
//
void CPnt::InvTransform(const CFrame &frame)
{
    float c = (float)cos(frame.fThita);
    float s = (float)sin(frame.fThita);

    CPnt pt(frame.x, frame.y);

    pt.x += x * c - y * s;
    pt.y += y * c + x * s;

    x = pt.x;
    y = pt.y;
}

//
//   判断一个给定的点是否“触碰”到该点。
//
bool CPnt::PointHit(const CPnt &pt, float fDistGate) const
{
    return (DistanceTo(pt) < fDistGate);
}

#ifdef _MFC_VER

//
//   “拷备”构造函数。
//
void CPnt::operator=(const CPoint &point)
{
    x = (float)point.x;
    y = (float)point.y;
}

//
//   重载操作符 "==".
//
bool CPnt::operator==(const CPoint &point) const
{
    return ((x == (float)point.x) && (y == (float)point.y));
}

//
//   重载操作符 "!=".
//
bool CPnt::operator!=(const CPoint &point) const
{
    return (x != (float)point.x || y != (float)point.y);
}

void CPnt::Dump()
{
    TRACE(_T("%d\t%f\t%f\t%f\t%f\n"), id, x, y, r, a);
}

//
//   在屏幕上绘制该点.
//   注：如果线宽(nLineWidth)不为零，表示需要画空心圆。
//
void CPnt::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF color, int nPointSize, int nLineWidth)
{
    bool bSolidFill = true;

    // 如果线宽不为零，表示需要画空心圆
    if (nLineWidth > 0)
    {
        bSolidFill = false;
    }

    CPnt pt(x, y);

    CPoint pnt1 = ScrnRef.GetWindowPoint(pt);

    CRect r(pnt1.x - nPointSize, pnt1.y - nPointSize, pnt1.x + nPointSize, pnt1.y + nPointSize);

    CPen Pen;
    if (bSolidFill)
        Pen.CreatePen(PS_SOLID, 1, color);
    else
        Pen.CreatePen(PS_SOLID, nLineWidth, color);

    CPen *pOldPen = pDC->SelectObject(&Pen);

    CBrush Brush(color);
    CBrush *pOldBrush;

    if (bSolidFill)
        pOldBrush = pDC->SelectObject(&Brush);
    else
        pOldBrush = (CBrush *)pDC->SelectStockObject(NULL_BRUSH);

    pDC->Ellipse(&r);

    pDC->SelectObject(pOldPen);
    pDC->SelectObject(pOldBrush);
}
#elif defined QT_VERSION

CPnt::CPnt(const QPoint &point)
{
    x = (float)point.x();
    y = (float)point.y();
    a = 0;
    r = 0;
    id = 0;
}

//
//   “拷备”构造函数。
//
void CPnt::operator=(const QPoint &point)
{
    x = (float)point.x();
    y = (float)point.y();
}

//
//   重载操作符 "==".
//
bool CPnt::operator==(const QPoint &point) const
{
    return ((x == (float)point.x()) && (y == (float)point.y()));
}

//
//   重载操作符 "!=".
//
bool CPnt::operator!=(const QPoint &point) const
{
    return (x != (float)point.x() || y != (float)point.y());
}

void CPnt::Dump()
{
    //	TRACE(_T("%d\t%f\t%f\t%f\t%f\n"), id, x, y, r, a);
}

//
//   在屏幕上绘制该点.
//   注：如果线宽(nLineWidth)不为零，表示需要画空心圆。
//
void CPnt::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor color, int nPointSize, int nLineWidth,
                float mmSize)
{
    bool bSolidFill = true;

    // 如果线宽不为零，表示需要画空心圆
    if (nLineWidth > 0)
    {
        bSolidFill = false;
    }

    CPnt pt(x, y);

    QPoint pnt1 = ScrnRef.GetWindowPoint(pt);

    // 如果给定的mmSize大于零，则启用自动缩放模式(即采用给定的点的物理半径，根据比例尺自动计算显示点的半径，但最终半径点数不得小于nPointSize)
    if (mmSize > 0)
    {
        int nRadius = mmSize / 1000.0f * ScrnRef.m_fRatio;

        if (nRadius < nPointSize)
            nRadius = nPointSize;

        nPointSize = nRadius;
    }

    QRect r(pnt1.x() - nPointSize, pnt1.y() - nPointSize, 2 * nPointSize, 2 * nPointSize);

    QPen Pen(color);
    if (bSolidFill)
        Pen.setWidth(1);
    else
        Pen.setWidth(nLineWidth);

    QBrush Brush(color);
    if (!bSolidFill)
        Brush.setStyle(Qt::NoBrush);

    pPainter->setPen(Pen);
    pPainter->setBrush(Brush);
    pPainter->drawEllipse(r);
}
#endif
