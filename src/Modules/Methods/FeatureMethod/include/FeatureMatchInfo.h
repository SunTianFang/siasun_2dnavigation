#ifndef FEATUREMATCHINFO_H
#define FEATUREMATCHINFO_H

#include "MatchInfo.h"
#include "PointMatchPair.h"
#include "LineMatchPair.h"
#include "PointMatchList.h"

class DllExport CFeatureMatchInfo : public CMatchInfo
{
  public:
    CFeatureMatchInfo()
    {
        type_ = 1;
    }
    virtual CMatchInfo *Duplicate() const
    {
        CFeatureMatchInfo *p = new CFeatureMatchInfo;
        *p = *this;
        return p;
    }
    vector<CPointMatchPair>  vecPointPair_;//point匹配对
    vector<CLineMatchPair>   vecLinePair_;//linePair_
    vector<CPnt>     vecReflectors_;//识别到的反光板
    vector<CLine>    vecLines_;//识别到的直线
    void Clear()
    {
        vecPointPair_.clear();
        vecLinePair_.clear();
        vecReflectors_.clear();
        vecLines_.clear();
    }
};
#endif // FEATUREMATCHINFO_H
