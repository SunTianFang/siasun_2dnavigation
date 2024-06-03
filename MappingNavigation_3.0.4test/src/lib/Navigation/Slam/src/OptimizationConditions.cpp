#include <stdafx.h>
#include "OptimizationConditions.h"

///////////////////////////////////////////////////////////////////////////////

bool CConstraint::LoadBinary(FILE *fp)
{
    int n[2];
    float f[3];

    // 读入约束对的两个位姿编号
    if (!fread(n, sizeof(int), 2, fp))
        return false;

    // 读入相对观测位姿
    if (!fread(f, sizeof(float), 3, fp))
        return false;

    nStepId1 = n[0];
    nStepId2 = n[1];
    pstObserved.x = f[0];
    pstObserved.y = f[1];
    pstObserved.fThita = f[2];

    return true;
}

bool CConstraint::SaveBinary(FILE *fp)
{
    int n[2] = {nStepId1, nStepId2};
    float f[3] = {pstObserved.x, pstObserved.y, pstObserved.fThita};

    // 写入约束对的两个位姿编号
    if (!fwrite(n, sizeof(int), 2, fp))
        return false;

    // 写入相对观测位姿
    if (!fwrite(f, sizeof(float), 3, fp))
        return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////

//
//   在约束条件向量中寻找指定的约束。
//
int COptimizationConditions::FindConstraint(int nFromStepId, int nToStepId)
{
    for (int i = 0; i < (int)m_Constraints.size(); i++)
    {
        CConstraint &c = m_Constraints[i];
        if (c.nStepId1 == nFromStepId && c.nStepId2 == nToStepId)
            return i;      // 找到了，返回其序号
    }

    return -1;        // 没有找到
}

//
//   根据给定的参数添加约束条件(如该位姿对的约束已存在，则对该约束的观测位姿进行替换)。
//
bool COptimizationConditions::AddConstraint(int nFromStepId, int nToStepId, const CPosture& pst)
{
    int idx = FindConstraint(nFromStepId, nToStepId);

    // 如果对应于该位姿对的约束已经存在，在此应将其对应的观测位姿替换为给定值(pst)
    if (idx >= 0)
        m_Constraints[idx].pstObserved = pst;

    // 如果不存在，则添加新约束
    else
    {
        CConstraint c(nFromStepId, nToStepId, pst);
        m_Constraints.push_back(c);
    }
    return true;
}

void COptimizationConditions::DeleteConstraint(int index)
{
    if (index >= m_Constraints.size())
        return;

    m_Constraints.erase(m_Constraints.begin() + index);
}

// 清除所有的约束条件
void COptimizationConditions::Clear()
{
    m_Constraints.clear();
    m_FixedPoses.clear();
}

bool COptimizationConditions::LoadBinary(FILE *fp)
{
    m_Constraints.clear();
    m_FixedPoses.clear();

    // 先读取约束数量
    int nCountConstraints;
    if (fread(&nCountConstraints, sizeof(int), 1, fp) == 1)
    {
        m_Constraints.resize(nCountConstraints);

        // 读取所有约束数据
        for (int i = 0; i < nCountConstraints; i++)
        {
            if (!m_Constraints[i].LoadBinary(fp))
                return false;
        }
    }

    // 读取固定位姿数量
    int nCountFixedPoses;
    if (fread(&nCountFixedPoses, sizeof(int), 1, fp) == 1)
    {
        m_FixedPoses.resize(nCountFixedPoses);

        // 读取所有约束数据
        for (int i = 0; i < nCountFixedPoses; i++)
            if (fread(&(m_FixedPoses[i]), sizeof(int), 1, fp) != 1)
                return false;
    }

    return true;
}

bool COptimizationConditions::SaveBinary(FILE *fp)
{
    // 写入约束数量
    int nCountConstraints = m_Constraints.size();
    if (fwrite(&nCountConstraints, sizeof(int), 1, fp) != 1)
        return false;

    // 写入所有约束数据
    for (int i = 0; i < (int)m_Constraints.size(); i++)
        if (!m_Constraints[i].SaveBinary(fp))
            return false;

    // 写入固定位姿数量
    int nCountFixedPoses = m_FixedPoses.size();
    if (fwrite(&nCountFixedPoses, sizeof(int), 1, fp) != 1)
        return false;

    // 写入所有约束数据
    for (int i = 0; i < (int)m_FixedPoses.size(); i++)
        if (fwrite(&(m_FixedPoses[i]), sizeof(int), 1, fp) != 1)
            return false;

    return true;
}
