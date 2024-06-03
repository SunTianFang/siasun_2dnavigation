#ifndef __MATCH_TAB_SET
#define __MATCH_TAB_SET

#include "PointMatchList.h"

#define TAB_ONE_SET_MAX_COUNT   64

///////////////////////////////////////////////////////////////////////////////

class CFeatureMatchTabSet
{
public:
	short count;
	CPointMatchList tab[TAB_ONE_SET_MAX_COUNT];

public:
	CFeatureMatchTabSet();

	// 清零一个匹配表集合
	void Clear() {count = 0;}

	// 取得一个匹配表集合中所含匹配表的数量
	int GetCount() {return count;}

	// 从一个匹配表集合中取得一个匹配表
	CPointMatchList GetTab(int i) {return tab[i];}

	// 向一个匹配表集合中加入一个新的匹配表
	bool Add(CPointMatchList& tabNew);

	// 在匹配表集合中查找一个匹配表，使它包含指定的两个匹配对
	int Search(CPointMatchPair& my_pair1, CPointMatchPair& my_pair2);

	// 寻找最优匹配表
	int FindBestMatch();
};
#endif
