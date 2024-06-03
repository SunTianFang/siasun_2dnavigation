#ifndef __MATCH_TAB
#define __MATCH_TAB

// 注意:可存储的点对数量大于实际参与匹配运算的数量

#define MAX_POINT_PAIR_COUNT      64      // 最大可存储点对的数量
#define MAX_POINT_MATCH_COUNT     10      // 最多可参与最小二乘匹配的直线段数量

#include "PointMatchPair.h"

class CLeastSquareMethod;

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchList"类的定义。
class CPointMatchList
{
public:
	int             m_nCount;
	CPointMatchPair m_PointPair[MAX_POINT_PAIR_COUNT];
        CTransform trans;

    int   m_dReFineTime;
	float agv_dist;
	float overlap_ratio;

public:
	CPointMatchList() {Clear();}

	// 清除表内数据
	void Clear() {m_nCount = 0;}

	// 取得表中匹配对的数量
	int GetCount() {return m_nCount;}

	// 为匹配表设置坐标变换关系
        void SetTrans(CTransform& _trans) {trans = _trans;}

	CPointMatchPair GetAt(int i) {return m_PointPair[i];}

	// 将一个匹配对加入到匹配表中
	bool Add(CPointMatchPair& Pair);

	// 将一个匹配对按局部点极径从小到大的顺序加入到匹配表中
	bool AddInOrder(CPointMatchPair& Pair);

	// 在一个匹配表中查找一个匹配对
	int Search(CPointMatchPair& my_pair);

	// 根据所提供的局部点在匹配表中查找其对应的匹配对
        int SearchByLocalPoint(CPnt& pnt);

	// 根据所提供的世界点在匹配表中查找其对应的匹配对
        int SearchByWorldPoint(CPnt& pnt);

	// 对匹配表中那些非”一对一“匹配的项进行移除
	void Filter();

	// 生成关于所有点对的最小二乘数据
	bool CreateLeastSquareData(CLeastSquareMethod* pLsm);

	// 根据匹配表计算两个点云之间的坐标变换关系
	bool FindTransform();

	// 计算两个点云之间的匹配程度参数
	void EvaluateTransform();

	// 计算两个点云之间的匹配程度参数
        bool EvaluateTransform(CTransform& Transform, float* fAverageErr, float* pMatchingScore);

        void ReFindTransform();

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
