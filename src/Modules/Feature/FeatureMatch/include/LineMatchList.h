#ifndef __CLineMatchList
#define __CLineMatchList

// 注意:可存储的线段对数量大于实际参与匹配运算的数量

#define MAX_LINE_PAIR_COUNT       256      // 最大可存储线段对的数量
#define MAX_LINE_MATCH_COUNT      10      // 最多可参与最小二乘匹配的直线段数量
#define MEAN_ERROR_EVALUATE   250.0f      // 250mm

#include "LineMatchPair.h"

class CLeastSquareMethod;

///////////////////////////////////////////////////////////////////////////////
//   "CLineMatchList"类的定义。
class CLineMatchList
{
public:
    int            m_nCount;                          // 线段匹配对数量
    int            m_nFootInLineCount;
    CTransform trans;
    float          m_fScore;
public:
        CLineMatchPair m_LinePair[MAX_LINE_PAIR_COUNT];  // 匹配表
	CLineMatchList() {Clear();}

    // 清除表内数据
    void Clear() {m_nCount = 0;m_nFootInLineCount = 0;}

	// 取得表中匹配对的数量
	int GetCount() {return m_nCount;}

	// 取得匹配表的指定荐
        CLineMatchPair GetAt(int i) {return m_LinePair[i];}

	// 将一个匹配对加入到匹配表中
	bool Add(const CLineMatchPair& Pair);

    bool AddInOrder(const CLineMatchPair& Pair);

    // 在一个匹配表中查找一个匹配对
    int Search(const CLineMatchPair& Pair);

	// 根据所提供的局部直线段在匹配表中查找其对应的匹配对
        int SearchByLocalLine(const CLine& ln);

	// 根据所提供的世界直线段在匹配表中查找其对应的匹配对
        int SearchByWorldLine(const CLine& ln);

	// 对匹配表中那些非”一对一“匹配的项进行移除
	void Filter(bool bDeleteParallel = false);

	// 判断是否匹配表中的所有匹配对都是平行线
	bool AllParallel();

    void EvaluateTransform();

    // 生成关于所有线段匹配对的最小二乘数据
    bool CreateLeastSquareData(CLeastSquareMethod* pLsm);

    void SortByFoot();

    void SortLines();

    void SortByLength();

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
