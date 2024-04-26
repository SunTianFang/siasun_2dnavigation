#pragma once

#include "Geometry.h"
#include "TimeStamp.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CStampedPos".
class CStampedPos : public CPosture, public CTimeStamp
{
  public:
    CStampedPos() {}
    CStampedPos(const CPosture &pos, const CTimeStamp &timesp) : CPosture(pos), CTimeStamp(timesp){};

    CStampedPos(const CStampedPos &other)
    {
        *this = other;
    }
#if 0
    CStampedPos &operator=(const CStampedPos &other)
    {
        GetPostureObject() = other.GetPostureObject();
        m_dwTimeStamp = other.m_dwTimeStamp;
        return *this;
    }
#endif
};
