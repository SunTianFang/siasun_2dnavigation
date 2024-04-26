#ifndef __CLineFeatureSet
#define __CLineFeatureSet

#include "ScanPoint.h"
#include "LineFeature.h"
#include "LineFeatureCreateParam.h"

#include <vector>

using namespace std;

//#define MAX_DIST_POINT_TO_POINT 80    // max distance for a cloud of points (grouping) 500
//#define MAX_SIGMA 30                  // max sigma value for a line
//#define MAX_SIGMA_RATIO 200
//#define MIN_POINTS_ON_LINE 15     // 直线段所含的最少点数  50
//#define MIN_LINE_LEN 300          // 直线段的最小长度450
//#define MAX_LINE_MERGE_DIST 80    // 两条平行直线的间距小于此值时可认为是一条直线，可以合并
//#define MAX_LINE_MERGE_ANGLE (5 * PI / 180)       // 两条直线的夹角小于此值时可认为是相互平行
//#define MIN_SCAN_TO_LINE_ANGLE (10 * PI / 180)    // 扫描线与直线特征所成夹角的最小值

class  CScan;

//
//   CLineFeatureSet描述扫描中的直线段。
//
class DllExport CLineFeatureSet : public vector<CLineFeature *>
{
  private:
    CRectangle m_rect;

  private:
    void UpdateCoveringRect();

    void SplitPoints(CScanPoint *sp, long lStart, long lEnd);

    // 将那些共线且相连的线段合并
    void LineScanMergeLines(CScan *scan, long *lineNum);

    // 删除那些扫描角不佳的直线特征
    void RemoveBadLines(const CPosture &pstScanner, const CScan &scan);

  public:
    CLineFeatureCreationParam m_Param;    // 直线生成参数
    CPosture m_pstScanner;                // 激光头参考姿态
    vector<CPnt> m_ptCorners;

    // 根据直线特征类型分配空间
    virtual CLineFeature *NewLineFeature(int nType);

  public:
    CLineFeatureSet(const CScan &scan);

    CLineFeatureSet();

    ~CLineFeatureSet();

    // 重载“=”操作符
    void operator=(const CLineFeatureSet &Obj);

    // 为特征集合分配空间
    bool Create(int nNum);

    // 设置直线性特征生成参数
    void SetCreationParam(CLineFeatureCreationParam *pParam);

    // 设置扫检测到这些直线特征时的激光头姿态，以便计算各条直线特征的观测方向
    void SetDetectPosture(const CPosture &pstDetect);

    // 从一个扫描集中抽取其有所有直线段
    bool CreateFromScan(const CScan &scan);

    // 根据当前姿态、最大扫描半径和直线模型来生成直线特征集合
    bool CreateFromLines(CPosture &pst, float fMaxRange, int nNumOfLines, CLine *pLines);

    bool CreateFromLinesSet(const CLineFeatureSet *WorldLineSet, CPosture &pstScanner, float fMaxScanDist,vector <CLine>& vecSpecialLineList,const CRectangle* pR=NULL);

    // 清除所有的直线扫描(CLineFeatureSet)
    void Clear();

    // 取得直线特征的数量
    int GetCount() const { return (int)size(); }

    // 取得指定的直线特征
    CLineFeature &GetLineFeature(int nIdx) { return *at(nIdx); }

    // 分配内存并复制当前对象内容
    CLineFeatureSet *Duplicate() const;

    // 将此另一个直线段集并入此直线段集中
    bool Merge(const CLineFeatureSet &LineScan);

	// 增加一个直线特征
	bool Add(const CLineFeature& LineFeature);

    // 重载操作符+=: 增加一个直线特征
    CLineFeatureSet& operator += (const CLineFeature *lineFeature);

    // 通过合并共线的线段来简化此直线段集合
    bool Simplify(float fMaxGapBetweenLines);

    // 删除指定的线段
    bool DeleteAt(int nIdx);

    // returns field of view of linescan in rad.
    float FieldOfView(CScan *scan);

    // returns total length of all line segments
    float TotalLength();

    // 去掉所有长度短于minLineLength的直线
    void LengthFilter(float minLineLength);

    // 判断直线扫描集是否包含指定的点
    bool ContainScanPoint(const CScanPoint &sp);

    // 移除位于指定区域内的线段
    void RemoveWithin(const CRectangle &r);

    // 将特征进行平移
    virtual void Move(float fX, float fY);

    // 将特征进行旋转
    virtual void Rotate(CAngle ang, CPnt ptCenter);

    // 从文本文件装入直线特征集合
    virtual bool LoadText(FILE *fp);

    // 将直线特征集合存到文本文件
    virtual bool SaveText(FILE *fp);

    // 从二进制文件装入直线特征集合
    virtual bool LoadBinary(FILE *fp);

    // 将直线特征集合存到二进制文件
    virtual bool SaveBinary(FILE *fp);

    // 取得最左点X坐标
    float LeftMost() { return m_rect.Left(); }

    // 取得最右点X坐标
    float RightMost() { return m_rect.Right(); }

    // 取得最上点Y坐标
    float TopMost() { return m_rect.Top(); }

    // 取得最下点Y坐标
    float BottomMost() { return m_rect.Bottom(); }

    // 取得整个直线集合的外阔尺寸
    // 取得覆盖区域
    CRectangle GetCoveringRect() const { return m_rect; }
};

/*
** Merges lines that are co-linear and connected.
*/
void LineScanMergeLines(CLineFeatureSet *ls, CScan *scan, long *newLineNumbers, CLineFeatureCreationParam *pParam);

#endif
