#ifndef __CDataRange
#define __CDataRange

#include <vector>
#include "ZTypes.h"

using namespace std;

class CDataRangeSet;

/////////////////////////////////////////////////////////////////////
// 关于"数据范围"的描述。
class DllExport CDataRange
{
  public:
    float from_;    // 数据起始处
    float to_;      // 数据终止处

  public:
    CDataRange(float from, float to)
    {
        from_ = from;
        to_ = to;
    }

    CDataRange() { Clear(); }

    // 清除范围
    void Clear() { from_ = to_ = 0; }

    // 取得对象的引用
    CDataRange &GetDataRangeObject() { return *this; }

    // 判断此范围是否合法
    bool IsValid() const { return from_ <= to_; }

    // 判断范围是否为空
    bool IsEmpty() const { return (from_ == to_); }

    // 判断此范围是否完全处在另一个范围的上一端
    bool IsAbove(const CDataRange &another) const { return (from_ > another.to_); }

    // 判断此范围是否完全处在另一个范围的下一端
    bool IsBelow(const CDataRange &another) const { return (to_ < another.from_); }

    // 判断一个数是否在该范围中
    bool Contain(float f) const { return (f >= from_ && f <= to_); }

    // 判断此范围是否与另一个范围有重叠区
    bool IsOverlap(const CDataRange &another) const
    {
        if (IsAbove(another) || IsBelow(another))
            return false;
        else
            return true;
    }

    // 重载操作符"+="，实现两个范围的合并(如果两个范围无重叠区，则无法合并)
    void operator+=(const CDataRange &another)
    {
        if (!IsOverlap(another))
            return;

        if (another.from_ < from_)
            from_ = another.from_;

        if (another.to_ > to_)
            to_ = another.to_;
    }

    // 重载操作符"^="，实现两个范围的"与运算"(如果两个范围无重叠区，则清空本对象)
    void operator^=(const CDataRange &another)
    {
        if (!IsOverlap(another))
            Clear();

        from_ = max(from_, another.from_);
        to_ = min(to_, another.to_);

        if (from_ >= to_)
            Clear();
    }

    CDataRange operator ^ (const CDataRange &another)
    {
        CDataRange temp = *this;
        temp ^= another;
        return temp;
    }

    // 从本范围中去除另外一个范围，得到一个范围集合
    CDataRangeSet operator-(const CDataRange &another) const;
};

/////////////////////////////////////////////////////////////////////
// 定义“数据范围段的集合”。
class DllExport CDataRangeSet : public vector<CDataRange>
{
  private:
  public:
    // 直接构造成仅含有一个数据范围
    CDataRangeSet(const CDataRange &range);

    // 直接按另一集合构造
    CDataRangeSet(const CDataRangeSet &another);

    CDataRangeSet() {}

    // 取得对象的引用
    CDataRangeSet &GetDataRangeSetObject() { return *this; }

    // 添加一个范围
    void operator+=(const CDataRange &range);

    // 添加一个范围集合
    void operator+=(const CDataRangeSet &another);

    // 实现集合与元素的加法
    CDataRangeSet operator+(const CDataRange &range);

    // 实现集合与集合的加法
    CDataRangeSet operator+(const CDataRangeSet &another);

    // 从集合中去除一个范围
    void operator-=(const CDataRange &range);

    // 从集合中去除一个范围集合
    void operator-=(const CDataRangeSet &another);

    // 实现集合与元素的减法
    CDataRangeSet operator-(const CDataRange &range);

    // 实现集合与集合的减法
    CDataRangeSet operator-(const CDataRangeSet &another);

    // 实现本范围集合与另外一个范围之间的“与”运算
    void operator^=(const CDataRange &range);

    // 实现本范围集合与另外一个范围集合之间的“与”运算
    void operator^=(const CDataRangeSet &another);

    // 实现集合与元素的“与运算”
    CDataRangeSet operator^(const CDataRange &range);

    // 实现集合与集合的“与运算”
    CDataRangeSet operator^(const CDataRangeSet &another);

    // 赋值
    void operator=(const vector<CDataRange> &another);

    // 判断一个数是否在该范围集合中
    bool Contain(float f) const;

    // 对一个“范围集合”进行标准化
    void Normalize();
};
#endif
