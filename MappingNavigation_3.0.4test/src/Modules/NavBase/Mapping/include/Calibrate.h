
#pragma once

#include <stdio.h>
#include <deque>
#include <pthread.h>
#include <atomic>
#include "ZTypes.h"
#include "Geometry.h"
#include "MagicSingleton.h"

#include "LaserMapping.h"



namespace mapping {

class CCalibrate : public CLaserMapping
{

private:
    CCalibrate();
    ~CCalibrate();

    friend MagicSingleton2<CCalibrate>;

private:


    bool StartRecordDx();

public:

    bool WriteLaserParam(int res,double *lasertf);

    bool StartCalibration();

    bool StopCalibration();

    bool StopRecordDx();


};
}





using CalibrateSingleton = MagicSingleton2<mapping::CCalibrate>;
