//
//   The interface of class "CLaserMapping".
//

#pragma once

#include <stdio.h>
#include <deque>
#include <pthread.h>
#include <atomic>
#include "ZTypes.h"
#include "Geometry.h"
#include "MagicSingleton.h"

namespace mapping {

class CLaserMapping
{
public:
    bool            m_bStarted;
    unsigned int    m_nStartTime;
    double          m_dResolution;
    bool            m_bFirstTime;
    std::atomic_uchar   m_aWorkMode;
    atomic_ullong m_CurTimeStamp;
    atomic_ullong m_PreTimeStamp;
    atomic_bool m_aFileSaving;
    CPosture    m_CurOdomPst;
    CPosture    m_PreOdomPst;

public:
    HANDLE     m_hKillThread;       // Handle of "Kill thread" event
    HANDLE     m_hThreadDead;       // Handle of "Thread dead" event
    pthread_t  m_pMappingThread;

public:
    CLaserMapping();
    ~CLaserMapping();

    friend MagicSingleton2<CLaserMapping>;

private:
    bool Stop();

    bool UpdateOdometry();

    bool UpdateScan();

    bool WriteLaserParm(unsigned int nStartTime);

    bool WriteScanData();

public:
    void SupportRoutine();

    bool StartMapping();

    bool StopMapping();

    bool SaveFile();

    void SetWorkMode(unsigned char uMode);

};

} // namespace mapping

using LaserMappingSingleton = MagicSingleton2<mapping::CLaserMapping>;
