#ifndef __CPointFeatureSet
#define __CPointFeatureSet

#include <stdio.h>
#include <vector>
#include "PointFeature.h"

using namespace std;

typedef CPointFeature *CPtrPointFeature;

//////////////////////////////////////////////////////////////////////////////
//   定义“点特征集合”类。
class DllExport CPointFeatureSet : public vector<CPointFeature *>
{
  private:
    CRectangle m_rect;
    float **m_pDistCache;    // 用于存储点与点之间距离的数据缓冲区
    bool m_bPreComputeDist;

  protected:
    // 根据所提供的特征类型分配空间
    virtual CPointFeature *NewPointFeature(int nSubType);

    // 更新边界值
    void UpdateCoveringRect();

    // 清除内部的各点之间的距离值
    void ClearDistanceCache(bool force = false);

  public:
    CPointFeatureSet();
    ~CPointFeatureSet();

    // “拷贝”构造函数
    CPointFeatureSet(const CPointFeatureSet &another);

    // 赋值
    CPointFeatureSet &operator=(const CPointFeatureSet &another);

    // 清除集合
    virtual void Clear();

    // 根据极坐标计算出点的迪卡尔坐标(在无速度输入的情况下)
    void UpdateCartisian();

    // 取得集合内点的数量
    int GetCount() { return (int)size(); }

    // 使能/禁止各点间距离的预先计算
    void EnableDistanceCache(bool enable) { m_bPreComputeDist = enable; }

    // 向集合内添加一个新点
    CPointFeatureSet &operator+=(CPointFeature *pNewFeature);

    // 向集合内添加一个新点
    CPointFeatureSet &operator+=(const CPointFeature &newFeature);

    // 将另一个集合并入本集合中
    CPointFeatureSet &operator+=(const CPointFeatureSet &another);

    // 删除指定序号的点
    bool DeleteAt(int nIdx);

    int CreateFromOtherSet(CPointFeatureSet &WorldLayer, CPosture &pstScanner, float fMaxScanDist ,vector <CPnt>&  vecSpecialPntList,const CRectangle* pR=NULL);

    // 对所有的点特征按极角从小到大的顺序进行排序
    //    void SortByAngle();

    // 取得覆盖区域
    CRectangle GetCoveringRect() const { return m_rect; }

    // 为各点之间的距离分配存储空间
    bool CreateDistanceCache(bool force = false);

    // 取得i, j两点之间的距离
    float PointDistance(int i, int j);

    // 取得该点集的中心点
    CPnt GetMassCenter();

    // By Sam: 对所有反光板进行坐标变换
    void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

    bool  CalculateCovarianceMatrix(CPointFeatureSet vecPoints,Matrix2d &cov);

    // 从文本文件中装入特征集合数据
    virtual bool LoadText(FILE *fp);

    // 将特征集合数据保存到文本文件中
    virtual bool SaveText(FILE *fp, int iSize = 0);

    // 从二进制文件中装入特征集合数据
    virtual bool LoadBinary(FILE *fp);

    // 将特征集合数据保存到二进制文件中
    virtual bool SaveBinary(FILE *fp);
};
#endif
