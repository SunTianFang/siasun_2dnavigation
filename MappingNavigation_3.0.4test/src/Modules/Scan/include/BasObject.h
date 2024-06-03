#pragma once

#include "ObjectElement.h"

class CScan;

///////////////////////////////////////////////////////////////////////////////
// 定义“基本物体”类型。
class DllExport CBasObject : public CObjectElements
{
  private:
    bool visible;

  public:
    int m_nTempId;       // 该物体的模板编号
    int m_nObjId;        // 该物体的物体编号
    std::string name;    // 该物体的名称

  private:
    // 在当前物体的基础上，假定从指定的扫描姿态进行扫描，计算当增加一个物体元素e时会成生的扫描结果段
    void AddScanElement(const CPosture &pstScanner, const CObjectElement &e);

  public:
    CBasObject();
    CBasObject(const CBasObject &another) { *this = another; }
    ~CBasObject() { Clear(); }

    CBasObject &GetBasObject() { return *this; }

    void Clear();

    // 生成一个副本
    virtual CBasObject *Duplicate();

    // 重载“=”操作符
    CBasObject &operator=(const CBasObject &another);

    // 向物体模板中加入一个新的元素(直线/圆，等等)
    CBasObject &operator+=(const CObjectElement &e);

    // 将另一个物体模板加入到本物体模板中
    CBasObject &operator+=(const CBasObject &another);

    // 向物体模板中加入一个新的圆形物体
    CBasObject &operator+=(const CCircle &circle);

    // 向物体模板中加入一个新的多边形物体
    CBasObject &operator+=(const CPolyRegion &polygon);

    // 设置对象可见/不可见
    void SetVisible(bool v) { visible = v; }

    // 判断对象可见/不可见
    bool IsVisible() { return visible; }

    // 假定以pstScanner作为扫描器姿态，产生对此物体的扫描结果result
    void CreateScanResult(const CPosture &pstScanner, CBasObject &result);

    // 假定以pstScanner作为扫描器姿态，产生对此物体的扫描结果点云pointCloud
    void CreateScanPointCloud(const CPosture &pstScanner, float startAngle, float endAngle,
                              int scanLineCount, CScan &pointCloud);

    // 进行坐标正变换
    virtual void Transform(const CFrame &frame);

    // 进行坐标逆变换
    virtual void InvTransform(const CFrame &frame);

    // 从文本文件中装入单个物体模板
    virtual bool LoadText(FILE *fp);

    // 将物体模板保存到文本文件中
    virtual bool SaveText(FILE *fp);

    // 从二进制文件中装入单个物体模板
    virtual bool LoadBinary(FILE *fp);

    // 将物体模板保存到二进制文件中
    virtual bool SaveBinary(FILE *fp);

#ifdef _MFC_VER
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth = 1);
#elif defined QT_VERSION
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                      int nLineWidth = 1);
#endif
};
