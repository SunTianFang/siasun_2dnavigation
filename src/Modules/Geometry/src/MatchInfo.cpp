#include <stdafx.h>
#include "MatchInfo.h"

///////////////////////////////////////////////////////////////////////////////

CVectMatchInfo::~CVectMatchInfo()
{
    Clear();
}

void CVectMatchInfo::Clear()
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i) != NULL)
            delete at(i);

    clear();
}

//
//   加入一条新的测试结果。
//
bool CVectMatchInfo::Append(const CMatchInfo &info)
{
    CMatchInfo *p = info.Duplicate();
    if (p == NULL)
        return false;

    push_back(p);
    return true;
}
