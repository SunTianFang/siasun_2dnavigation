#include <stdafx.h>
#include "LineElement.h"
#include "CircleElement.h"
#include "BasObject.h"
#include "Scan.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   “CObjTemplate”类的实现。

CBasObject::CBasObject()
{
    visible = true;
    m_nTempId = -1;
    m_nObjId = -1;
}

void CBasObject::Clear()
{
    for (int i = 0; i < (int)size(); i++)
        delete at(i);
    clear();
}

//
//   生成一个副本。
//
CBasObject *CBasObject::Duplicate()
{
    CBasObject *p = new CBasObject;
    if (p == NULL)
        return NULL;

    *p = *this;
    return p;
}

//
//   重载“=”操作符。
//
CBasObject &CBasObject::operator=(const CBasObject &another)
{
    Clear();
    resize(another.size());

    for (int i = 0; i < (int)another.size(); i++)
        at(i) = another[i]->Duplicate();

    visible = another.visible;
    m_nTempId = another.m_nTempId;
    m_nObjId = another.m_nObjId;
    name = another.name;

    return *this;
}

//
//   向物体中加入一个新的元素(直线/圆，等等)。
//
CBasObject &CBasObject::operator+=(const CObjectElement &e)
{
    push_back(e.Duplicate());
    return *this;
}

//
//   将另一个物体中加入到本物体中。
//
CBasObject &CBasObject::operator+=(const CBasObject &another)
{
    for (int i = 0; i < (int)another.size(); i++)
        *this += *another[i];

    return *this;
}

//
//   向物体模板中加入一个新的圆形物体。
//
CBasObject &CBasObject::operator+=(const CCircle &circle)
{
    push_back(new CCircleElement(circle));
    return *this;
}

//
//   向物体模板中加入一个新的多边形物体。
//
CBasObject &CBasObject::operator+=(const CPolyRegion &polygon)
{
    for (int i = 0; i < (int)polygon.m_nCount - 1; i++)
    {
        CLine line(polygon.m_pVertex[i], polygon.m_pVertex[i + 1]);
        CLineElement *lineObj = new CLineElement(line);
        push_back(lineObj);
    }

    return *this;
}

//
//   在当前物体的基础上，假定从指定的扫描姿态进行扫描，计算当增加一个物体元素e时会成生的扫描结果段。
//
void CBasObject::AddScanElement(const CPosture &pstScanner, const CObjectElement &e)
{
    // 先移除本物体中所有的非直线元素
    for (int i = (int)size() - 1; i >= 0; i--)
    {
        if (at(i)->m_nType != ELEMENT_TYPE_LINE)
        {
            delete at(i);
            erase(begin() + i);
        }
    }

    // 如果新元素不是直线元素，则不进行任何处理
    if (e.m_nType != ELEMENT_TYPE_LINE)
        return;

    // 取得指向obstacle直线的指针
    const CLineElement *obstacle = dynamic_cast<const CLineElement *>(&e);
    if (obstacle == NULL)
        return;

    // 定义临时工作变量
    CBasObject temp;
    CLine obsLine(obstacle->m_line);

    // 将obsLine调整为从pstScanner看来是逆时针方向的
    float ang1, ang2;
    AdjustLine(obsLine, pstScanner, ang1, ang2);

    CMultiSegLine obstacleLine(obsLine);

    // 依次生成遮挡结果
    for (int i = 0; i < (int)size(); i++)
    {
        CLineElement *line = dynamic_cast<CLineElement *>(at(i));
        if (line == NULL)
            continue;

        CBasObject remain;    // 当前直线被obstacle线遮挡后剩余的部分
        CMultiSegLine obs;    // obstacle线被当前直线遮挡后剩余的部分(以多段线的方式保存)

        // 计算当前元素被obstacle元素遮挡的情况，取得相关的遮挡结果remain，obs
        line->ApplyObstacle(*obstacle, pstScanner, remain, obs);

        // 依次将当前直线被挡后剩余的部分加入到temp中
        temp += remain;

        // obstacle直线中，只有obs中剩余的部分才会继续保留下来
        obstacleLine ^= obs;
    }

    // 最后，将obstaleLine的剩余部分也加进来
    for (int i = 0; i < (int)obstacleLine.size(); i++)
    {
        CLine line;
        if (obstacleLine.GetSegment(i, line))
            temp += CLineElement(line);
    }

    // 取得结果
    *this = temp;
}

//
//   假定以pstScanner作为扫描器姿态，产生对此物体的扫描结果result。
//
void CBasObject::CreateScanResult(const CPosture &pstScanner, CBasObject &result)
{
    result.Clear();

    if (size() == 0)
        return;

    result += *at(0);
    if (size() == 1)
        return;

    for (int i = 1; i < (int)size(); i++)
        result.AddScanElement(pstScanner, *at(i));
}

//
//   假定以pstScanner作为扫描器姿态，产生对此物体的扫描结果点云pointCloud。
//
void CBasObject::CreateScanPointCloud(const CPosture &pstScanner, float startAngle, float endAngle,
                                      int scanLineCount, CScan &pointCloud)
{
    CBasObject scannedObj;

    CreateScanResult(pstScanner, scannedObj);

    // 为点云分配空间，并设置点云基本参数
//    pointCloud.Clear();
//    pointCloud.Create(scanLineCount);
    pointCloud.m_fStartAng = startAngle;
    pointCloud.m_fEndAng = endAngle;
    pointCloud.m_pstScanner = pstScanner;

    float angReso = (endAngle - startAngle) / scanLineCount;   // 角分辨率

    //angReso = 0.0029;   // By Jzz

    // 初始化点云，使所有点都在零点
    for (int i = 0; i < scanLineCount; i++)
    {
        CScanPoint &sp = pointCloud.m_pPoints[i];
        sp.r = 0;
        sp.a = 0;
        sp.x = sp.y = 0;
        sp.m_nIntensity = 0;
    }

    // 对扫描到的物体中的每一条直线，依次生成对应的扫描点云
    for (int i = 0; i < (int)scannedObj.size(); i++)
    {
        // 取得物体的下一个元素
        CLineElement *le = dynamic_cast<CLineElement *>(scannedObj[i]);

        // 目前，必须是直线元素
        if (le == NULL)
            continue;

        // 得到当前直线段
        CLine &line = le->m_line;

        // 构造对应于该直线两个端点的扫描线
        CLine startScanLine(pstScanner, line.m_ptStart);
        CLine endScanLine(pstScanner, line.m_ptEnd);

        // 计算该直线相对于激光器姿态的起始角和终止角
        CAngle ang1 = startScanLine.SlantAngle();
        CAngle ang2 = endScanLine.SlantAngle();

        // 计算从起始角到终止角的转角，并计算该直线段所对应的扫描线数
        CAngle angDiff = ang2 - ang1;
        int count = (int)(angDiff.m_fRad / angReso);

        // 计算对应此直线段的第一根扫描线在点云中的序号
        int j = (int)((ang1 - pstScanner.GetAngle() - startAngle).m_fRad / angReso);

        // 针对每一条直线，产生对应的扫描点
        for (int k = 0; k < count; k++)
        {
            CScanPoint &sp = pointCloud.m_pPoints[j + k];         // 扫描点
            CAngle scanAngle = startAngle + (j + k) * angReso;    // 扫描角

            CPnt pt;
            float dist;

            // 计算扫描线与目标直线的交点
            CLine scanLine(pstScanner, scanAngle + pstScanner.GetAngle(), 1000);
            if (scanLine.IntersectLineAt(line, pt, dist))
            {
                // jzz:注意，此处生成target点云的r，r值赋的是dist（距离），是传感器发射的射线与line交点的距离，因此是局部坐标系下描述的target点
                // 给r = dist，则生成的target是局部坐标系（激光坐标系）下描述的。{这里应该差一个外参（激光安装位姿）的变换，这里默认是（0,0,0)车体中心}
                // 由于source点云也是转到车体中心了，因此不用再转到laser外参下描述了。
                // 如果改为pt赋值，则为map坐标系下的点，生成的target是map坐标系。
                sp.r = dist;
                sp.a = scanAngle.m_fRad;
                sp.UpdateCartisian();
//                sp.r *= 1000; // 原程序中不合理之处，r单位为mm，而x/y单位为m-将来应修改
            }
        }
    }
}

//
//   进行坐标正变换。
//
void CBasObject::Transform(const CFrame &frame)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Transform(frame);
}

//
//   进行坐标逆变换。
//
void CBasObject::InvTransform(const CFrame &frame)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->InvTransform(frame);
}

//
//   从文本文件中装入单个物体模板。
//
bool CBasObject::LoadText(FILE *fp)
{
    Clear();

    int count;
    if (fscanf(fp, "%d\n", &count) != 1)
        return false;

    for (int i = 0; i < count; i++)
    {
        int type;
        if (fscanf(fp, "%d\t", &type) != 1)
            return false;

        // 根据元素类型读取
        switch (type)
        {
        case ELEMENT_TYPE_LINE:    // 直线
        {
            CLineElement line;
            if (!line.LoadText(fp))
                return false;

            *this += line;
        }
        break;

        case ELEMENT_TYPE_CIRCLE:    // 圆
        {
            CCircleElement circle;
            if (!circle.LoadText(fp))
                return false;

            *this += circle;
        }
        break;

        default:
            break;
        }
    }

    return true;
}

//
//   将物体模板保存到文本文件中。
//
bool CBasObject::SaveText(FILE *fp)
{
    fprintf(fp, "%d\n", size());

    for (int i = 0; i < (int)size(); i++)
    {
        int type = at(i)->m_nType;
        fprintf(fp, "%d\t", type);

        // 根据元素类型读取
        switch (type)
        {
        case ELEMENT_TYPE_LINE:    // 直线
            if (!((CLineElement *)at(i))->SaveText(fp))
                return false;
            break;

        case ELEMENT_TYPE_CIRCLE:    // 圆
            if (!((CCircleElement *)at(i))->SaveText(fp))
                return false;
            break;

        default:
            break;
        }

        fprintf(fp, "\n");
    }

    fprintf(fp, "\n");

    return true;
}

//
//   从二进制文件中装入单个物体模板。
//
bool CBasObject::LoadBinary(FILE *fp)
{
    Clear();

    int count;
    if (fread(&count, sizeof(int), 1, fp) != 1)
        return false;

    for (int i = 0; i < count; i++)
    {
        int type;
        if (fread(&type, sizeof(int), 1, fp) != 1)
            return false;

        // 根据元素类型读取
        switch (type)
        {
        case 1:    // 直线
        {
            CLineElement line;
            if (!line.LoadBinary(fp))
                return false;

            *this += line;
        }
        break;

        case 2:    // 圆
        {
            CCircleElement circle;
            if (!circle.LoadBinary(fp))
                return false;

            *this += circle;
        }
        break;

        default:
            break;
        }
    }

    return true;
}

//
//   将物体模板保存到二进制文件中。
//
bool CBasObject::SaveBinary(FILE *fp)
{
    int count = size();
    if (fwrite(&count, sizeof(int), 1, fp) != 1)
        return false;

    for (int i = 0; i < (int)size(); i++)
    {
        int type = at(i)->m_nType;
        if (fwrite(&type, sizeof(int), 1, fp) != 1)
            return false;

        // 根据元素类型读取
        switch (type)
        {
        case ELEMENT_TYPE_LINE:    // 直线
            if (!((CLineElement *)at(i))->SaveBinary(fp))
                return false;
            break;

        case ELEMENT_TYPE_CIRCLE:    // 圆
            if (!((CCircleElement *)at(i))->SaveBinary(fp))
                return false;
            break;

        default:
            break;
        }
    }

    return true;
}

#ifdef _MFC_VER
void CObjTemplate::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Plot(ScrnRef, pDC, crColor, nLineWidth);
}

#elif defined QT_VERSION

void CBasObject::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nLineWidth)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Plot(ScrnRef, pPainter, crColor, nLineWidth);
}

#endif
