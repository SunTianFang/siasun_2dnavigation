#include "stdafx.h"
#include <math.h>
#include "Geometry.h"
#include "Frame.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QColor>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   将点绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, float fCx, float fCy)
{
    CPnt::Rotate(fAng, fCx, fCy);
    fThita = CAngle::NormAngle(fThita + fAng);
}

//
//   将姿态绕指定的中心点进行旋转。
//
void CPosture::Rotate(float fAng, CPnt ptCenter)
{
    CPnt::Rotate(fAng, ptCenter);
    fThita = CAngle::NormAngle(fThita + fAng);
}

//
//   计算姿态角到一条射线的夹角。
//
CAngle CPosture::AngleToLine(const CLine &ln) const
{
    return (ln.m_angSlant - fThita);
}

void CPosture::RotatePos(float angle, float cx, float cy)
{
    float sina = (float)sin(angle), cosa = (float)cos(angle);
    float dx = x - cx, dy = y - cy;

    x = cx + dx * cosa - dy * sina;
    y = cy + dx * sina + dy * cosa;
    fThita += angle;
}

//
//   在当前姿态的基础上，根据给定的速度向量，推算出一定时段后的新姿态。
//
CPosture CPosture::Deduce(const vector_velocity &vel, float interval)
{
    float dx = vel.vel_x * interval;
    float dy = vel.vel_y * interval;
    float s = (float)sin(fThita);
    float c = (float)cos(fThita);

    CPosture pstNew;
    pstNew.x = x + dx * c - dy * s;
    pstNew.y = y + dy * c + dx * s;
    pstNew.fThita = fThita + vel.vel_angle * interval;

    return pstNew;
}

//
//   进行坐标正变换。
//
void CPosture::Transform(const CFrame &frame)
{
    CPnt::Transform(frame);
    fThita = CAngle::NormAngle(fThita - frame.fThita);
}

//
//   进行坐标逆变换。
//
void CPosture::InvTransform(const CFrame &frame)
{
    CPnt::InvTransform(frame);
    fThita = CAngle::NormAngle(fThita + frame.fThita);
}

//
//   从文本文件装入数据。
//
bool CPosture::LoadText(FILE *fp)
{
    if (fscanf(fp, "%f\t%f\t%f\t", &x, &y, &fThita) != 3)
        return false;

    return true;
}

//
//   将数据写入文本文件。
//
bool CPosture::SaveText(FILE *fp)
{
    fprintf(fp, "%f\t%f\t%f\t", x, y, fThita);
    return true;
}

//
//   从二进制文件装入数据。
//
bool CPosture::LoadBinary(FILE *fp)
{
    float f[3];
    if (fread(f, sizeof(float), 3, fp) != 3)
        return false;

    SetPosture(f[0], f[1], f[2]);
    return true;
}

//
//   将数据写入二进制文件。
//
bool CPosture::SaveBinary(FILE *fp)
{
    float f[3] = {x, y, fThita};
    if (fwrite(f, sizeof(float), 3, fp) != 3)
        return false;

    return true;
}

//#define SCANNER_ARROW_LEN          0.15f
//#define SCANNER_RADIUS             0.04f

#ifdef _MFC_VER

//
//   在屏幕上绘制此姿态。
//
void CPosture::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crLine, COLORREF crFill, int nCircleRadiusMm, int nArrowLenMm, int nWidth)
{
    CPen pen(PS_SOLID, nWidth, crLine);
    CPen *pOldPen = pDC->SelectObject(&pen);

    // 根据预先设定的物理尺寸，计算箭头长度和圆半径
    int nArrowLen = nArrowLenMm / 1000.0f * ScrnRef.m_fRatio;
    int nRadius = nCircleRadiusMm / 1000.0f * ScrnRef.m_fRatio;

    if (nArrowLen < 10)
        nArrowLen = 10;

    if (nRadius < 4)
        nRadius = 4;

    CPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());
    CPoint pntArrowTip;
    pntArrowTip.x = pnt.x + (int)(nArrowLen * cos(fThita));
    pntArrowTip.y = pnt.y - (int)(nArrowLen * sin(fThita));

    CRect r1(pnt.x - nRadius, pnt.y - nRadius, pnt.x + nRadius, pnt.y + nRadius);
    CPoint pntStart(pnt.x + nRadius, pnt.y);
    CPoint pntEnd(pnt.x + nRadius, pnt.y);
    //	pDC->Arc(&r1, pntStart, pntEnd);
    GetPntObject().Draw(ScrnRef, pDC, crColor, nRadius);

    pDC->MoveTo(pnt);
    pDC->LineTo(pntArrowTip);
    pDC->SelectObject(&pOldPen);
}

#elif defined QT_VERSION

//
//   在屏幕上绘制此姿态。
//
void CPosture::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crLine, QColor crFill, int nCircleRadiusMm, int nArrowLenMm,
                    int nWidth, QString str, QColor clrText)
{
    QPen pen(crLine);
    pen.setWidth(nWidth);
    pPainter->setPen(pen);

    QBrush brush(crFill);
    pPainter->setBrush(brush);

    // 根据预先设定的物理尺寸，计算箭头长度和圆半径
    int nArrowLen = nArrowLenMm / 1000.0f * ScrnRef.m_fRatio;
    int nRadius = nCircleRadiusMm / 1000.0f * ScrnRef.m_fRatio;

    if (nArrowLen < 10)
        nArrowLen = 10;

    if (nRadius < 4)
        nRadius = 4;

    QPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());
    QPoint pntArrowTip;
    pntArrowTip.setX(pnt.x() + (int)(nArrowLen * cos(fThita)));
    pntArrowTip.setY(pnt.y() - (int)(nArrowLen * sin(fThita)));

    QRect r1(pnt.x() - nRadius, pnt.y() - nRadius, 2 * nRadius, 2 * nRadius);
    pPainter->drawEllipse(r1);
    pPainter->drawLine(pnt, pntArrowTip);

    // 在此显示字串
    if (str != "" && nRadius >= 10)
    {
        pPainter->setPen(clrText);
//        QFont font("Microsoft YaHei", 11, QFont::Bold, true);
//        pPainter->setFont(font);
        pPainter->drawText(r1, Qt::AlignHCenter | Qt::AlignVCenter, str);
    }
}
#endif

vector_velocity EstimateVel(CPosture pst1, CPosture pst2, float interval)
{
    vector_velocity vel;

    float fThita = pst1.fThita;
    float c = (float)cos(fThita);
    float s = (float)sin(fThita);
    float dx = pst2.x - pst1.x;
    float dy = pst2.y - pst1.y;

    vel.vel_x = (dx * c + dy * s) / interval;
    vel.vel_y = (dy * c - dx * s) / interval;
    vel.vel_angle = (pst2.fThita - pst1.fThita) / interval;

    return vel;
}

CPosture operator+(const CPosture &pst1, const CPosture &pst2)
{
    CPosture pst = pst1;
    pst += pst2;

    return pst;
}

CPosture operator-(const CPosture &pst1, const CPosture &pst2)
{
    CPosture pst = pst1;
    pst -= pst2;

    return pst;
}
