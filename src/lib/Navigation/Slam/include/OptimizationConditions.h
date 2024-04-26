#ifndef __COptimizationConditions
#define __COptimizationConditions

#include <vector>
#include <map>
#include "Geometry.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////

// 定义两个帧之间的约束
class CConstraint
{
  public:
    int nStepId1;            // 源数据帧的ID号
    int nStepId2;            // 目标数据帧的ID号
    CPosture pstObserved;    // 从ID1观测到ID2的姿态

  public:
    CConstraint(int id1, int id2)
    {
        nStepId1 = id1;
        nStepId2 = id2;
    }

    CConstraint(int id1, int id2, const CPosture &pst)
    {
        nStepId1 = id1;
        nStepId2 = id2;
        pstObserved = pst;
    }

    CConstraint() {}

    bool LoadBinary(FILE *fp);
    bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////

class COptimizationConditions
{
  public:
    vector<CConstraint> m_Constraints;    // 添加的约束条件
    vector<int> m_FixedPoses;             // 哪些位姿是固定的

  private:
    int FindConstraint(int nFromStepId, int nToStepId);

  public:
    COptimizationConditions() {}

    // 根据给定的参数添加约束条件
    bool AddConstraint(int nFromStepId, int nToStepId, const CPosture &pst);

    // 删除指定的约束条件
    void DeleteConstraint(int index);

    // 清除所有的约束条件
    void Clear();

    bool LoadBinary(FILE *fp);
    bool SaveBinary(FILE *fp);

};
#endif
