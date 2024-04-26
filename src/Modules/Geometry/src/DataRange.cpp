#include <stdafx.h>
#include "DataRange.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   实现“数据范围”类。

//
//   从本范围中去除另外一个范围，得到一个范围集合。
//
CDataRangeSet CDataRange::operator - (const CDataRange &another) const
{
    CDataRangeSet rangeSet(*this);

    // 如果范围有重叠区，则可以进行去除处理
    if (IsOverlap(another))
    {
        // 如果本范围的起始处小于another的起始处
        if (from_ <= another.from_)
        {
            // 本范围会被截断，其起始处会被留下，剩下的第一段的终止处应变为another的起始处
            rangeSet[0].to_ = another.from_;

            // 如果本范围的终止处大于another的终止处，则它的后一段也会被留下
            if (to_ - another.to_ >= 0.001f)
                rangeSet += CDataRange(another.to_, to_);
        }
        // 如果本范围的起始处大于another的起始处，且其终止处大于another的终止处，则它的前一段会被截去
        else if (to_ > another.to_)
            rangeSet[0].from_ = another.to_;

        // 否则，全部被截去
        else
            rangeSet.clear();
    }

    for (int i = (int)rangeSet.size() - 1; i >= 0; i--)
    {
        if (rangeSet[i].IsEmpty())
            rangeSet.erase(rangeSet.begin() + i);
    }

    return rangeSet;
}

//////////////////////////////////////////////////////////////////////////////
//   实现“数据范围集合”类。

//
//   直接构造成仅含有一个数据范围。
//
CDataRangeSet::CDataRangeSet(const CDataRange &range)
{
    if (!range.IsEmpty())
        push_back(range);
}

CDataRangeSet::CDataRangeSet(const CDataRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
        if (!another[i].IsEmpty())
            push_back(another[i]);
}

//
//   向集合中添加一个新的范围(递归实现)。
//
void CDataRangeSet::operator +=(const CDataRange& range)
{
	push_back(range);
	Normalize();
}

//
//   添加一个范围集合。
//
void CDataRangeSet::operator +=(const CDataRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
		*this += another[i];
}

//
//   实现集合与元素的加法。
//
CDataRangeSet CDataRangeSet::operator + (const CDataRange& range)
{
    CDataRangeSet temp(*this);
	temp += range;

	return temp;
}

//
//   实现集合与集合的加法。
//
CDataRangeSet CDataRangeSet::operator + (const CDataRangeSet& another)
{
    CDataRangeSet temp(*this);
	temp += another;
	return temp;
}

//
//   从范围集合中去除一个范围。
//
void CDataRangeSet::operator -=(const CDataRange &range)
{
    if (range.IsEmpty())
        return;

    for (int i = 0; i < (int)size(); i++)
    {
        CDataRangeSet rangeSet = at(i) - range;
        switch (rangeSet.size())
        {
        case 0:
            // 如果结果范围为空，则删除该段
            erase(begin() + i);
            i--;
            break;

        case 1:
            // 如果结果范围仅有一段，则修改该段
            at(i) = rangeSet[0];
            break;

        case 2:
            // 如果结果范围有两段，则需要增加一段
            at(i) = rangeSet[0];
            insert(begin() + i + 1, rangeSet[1]);
            i++;
            break;

        default:
            // 不可能出现
            break;
        }
    }
}

//
//   从集合中去除一个范围集合。
//
void CDataRangeSet::operator -=(const CDataRangeSet &another)
{
    for (int i = 0; i < (int)another.size(); i++)
        *this -= another[i];
}

//
//   实现集合与元素的减法。
//
CDataRangeSet CDataRangeSet::operator-(const CDataRange &range)
{
    CDataRangeSet temp(*this);
    temp -= range;
    return temp;
}

//
//   实现集合与集合的减法。
//
CDataRangeSet CDataRangeSet::operator-(const CDataRangeSet &another)
{
    CDataRangeSet temp(*this);
    temp -= another;
    return temp;
}

//
//   实现本范围集合与另外一个范围之间的“与”运算。
//
void CDataRangeSet::operator ^= (const CDataRange &another)
{
    if (another.IsEmpty())
    {
        clear();
        return;
    }

    CDataRangeSet temp;
    for (int i = 0; i < (int)size(); i++)
        temp += at(i) ^ another;
    *this = temp;

    Normalize();
}

//
//   实现本范围集合与另外一个范围集合之间的“与”运算。
//
void CDataRangeSet::operator ^= (const CDataRangeSet &another)
{
    if (another.size() == 0)
    {
        clear();
        return;
    }

    CDataRangeSet temp;
    for (int i = 0; i < (int)another.size(); i++)
        temp += *this ^ another[i];
    *this = temp;

    Normalize();
}

//
//   实现集合与元素的“与运算”。
//
CDataRangeSet CDataRangeSet::operator^(const CDataRange &range)
{
    CDataRangeSet temp(*this);
    temp ^= range;
    return temp;
}

//
//   实现集合与集合的“与运算”。
//
CDataRangeSet CDataRangeSet::operator^(const CDataRangeSet &another)
{
    CDataRangeSet temp(*this);
    temp ^= another;
    return temp;
}

//
//   判断一个数是否在该范围集合中。
//
bool CDataRangeSet::Contain(float f) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).Contain(f))
			return true;

	return false;
}

//
//   对一个“范围集合”进行标准化。
//
void CDataRangeSet::Normalize()
{
	int nSize = size();
	if (nSize < 2)
		return;

	// 分配临时空间
	bool* bDelete = new bool[nSize];

	// 开始时，标明所有项都不删除
	for (int i = 0; i < nSize; i++)
		bDelete[i] = false;

	// 逐项进行合并
	for (int i = 0; i < nSize; i++)
	{
		// 跳过已删除的项
		if (bDelete[i])
			continue;

		for (int j = 0; j < nSize; j++)
		{
			// 跳过已删除的项
			if (bDelete[j] || (i == j))
				continue;

			// 如果两项可合并，则标明j项删除
            if (at(i).IsOverlap(at(j)))
			{
                at(i) += at(j);
				bDelete[j] = true;
			}
		}
	}

	// 实际删除所有已标记“delete”的项
	for (int i = size() - 1; i >= 0; i--)
	{
		if (bDelete[i])
			erase(begin() + i);
	}
	delete[]bDelete;

	// 重新排序
	for (int i = 0; i < (int)size()-1; i++)
	{
		for (int j = i + 1; j < (int)size(); j++)
		{
            if (at(i).IsAbove(at(j)))
			{
                CDataRange temp = at(i);
				at(i) = at(j);
				at(j) = temp;
			}
		}
	}
}

//
//   赋值。
//
void CDataRangeSet::operator = (const vector<CDataRange>& another)
{
	clear();

	for (int i = 0; i < (int)another.size(); i++)
		push_back(another[i]);
}
