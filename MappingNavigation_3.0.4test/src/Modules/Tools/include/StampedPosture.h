#ifndef __CStampedPosture
#define __CStampedPosture

#include "Geometry.h"
#include "TimeStamp.h"

//
//   定义具有时间戳的姿态。
//
class CStampedPosture : public CPosture, public CTimeStamp
{
public:
	CStampedPosture(const CPosture& pst, unsigned int uTime) : CPosture(pst), CTimeStamp(uTime)
	{
	}

	CStampedPosture() {}
};
#endif
