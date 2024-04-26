#ifndef __CLineFeature
#define __CLineFeature

#include <vector>
#include "Geometry.h"
#include "Feature.h"

// 特征类型
#define GENERIC_LINE_FEATURE 0         // 一般直线特征
#define SINGLE_SIDED_LINE_FEATURE 1    // 单侧直线特征

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。

class DllExport CLineFeature : public CFeature, public CLine
{
  public:
    long m_lStart;
    long m_lEnd;              // point numbers in scan
    float m_fSigma2;          // 匹配误差
    int m_nWhichSideToUse;    // 使用直线的哪一侧(1-前面;2-后面;3-两面)

    bool m_bCreateRef;       // 是否已生成参考点
    CPnt m_ptRef;            // 参考点的位置
    CPnt m_ptProjectFoot;    // 投影点的位置

  public:
    CLineFeature(const CPnt &ptStart, const CPnt &ptEnd);
    CLineFeature(const CLineFeature &other);
    CLineFeature();

    // 生成一个复本
    virtual CLineFeature *Duplicate() const;

    CLineFeature &GetLineFeatureObject() { return *this; }

    // 从文本文件中装入参数
    virtual bool LoadText(FILE *file);

    // 将参数写入到文本文件中
    virtual bool SaveText(FILE *file);

    // 从二进制文件中装入参数
    virtual bool LoadBinary(FILE *file);

    // 将参数写入到二进制文件中
    virtual bool SaveBinary(FILE *file);

    // 生成线段
    bool Create(const CPnt &ptStart, const CPnt &ptEnd);

    // 生成线段
    bool Create(const CLine &ln);

    // 根据给定的点云生成直线特征
    bool Create(const vector<CPnt> &points);

    // 设置扫检测到该直线特征时的激光头姿态，以便计算有效的观测朝向
    void SetDetectPosture(const CPosture &pstDetect);

    // 将该直线段投影到另一直线段，得到的投影线段存入lnResult
    bool ProjectToLine(CLineFeature &ln, CLineFeature &lnResult);

    // 将此线段与另外一条线段合并
    bool Merge(CLineFeature &Line2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines);

    // 将特征进行平移
    virtual void Move(float fX, float fY);

    // 将特征进行旋转
    virtual void Rotate(CAngle ang, CPnt ptCenter);

    // 进行坐标正变换
    virtual void Transform(const CFrame &frame);

    // 进行坐标逆变换
    virtual void InvTransform(const CFrame &frame);

    // 重载 "=="
    bool operator==(const CLineFeature lf) const;


};
#endif
