//
//   The interface of class "CRawMap".
//

#pragma once

#include <deque>
#include <mutex>
#include <atomic>
#include "RawScan.h"
#include "MagicSingleton.h"

#define RAW_MAP_CAPACITY_LOCAL      1000
#define RAW_MAP_CAPACITY_MAPPING    50000

namespace mapping {

class CRawMap
{
private:
    std::mutex map_mtx;
    std::atomic<unsigned long> m_aCount;
    std::atomic<unsigned long> m_aMaxCount;
    std::deque<sensor::CRawScan> raw_map;
    atomic_bool m_aFileSaving;
    unsigned int    m_nStartTime;
    int m_nWriteSensorCount;

private:
    CRawMap();

    friend MagicSingleton2<CRawMap>;

public:
    ~CRawMap();

    void Clear();

    bool AddRawScan(const sensor::CRawScan& pScan);

    bool GetFrontRawScan(sensor::CRawScan& pScan);

    bool GetBackRawScan(sensor::CRawScan& pScan);

    bool GetRawScans(std::deque<sensor::CRawScan> &pRawScans);

    void SetMaxCount(unsigned long max_count);

    unsigned int GetSize();

    //bool WriteLaserParm(FILE* pFile);

    bool WriteLaserParm(FILE* pFile,bool bPlsLaserWrite);

    bool WriteScanData(FILE* pFile);

    bool ReadScanData(FILE* pFile,int nVersion,int sensorCount, vector<int> lineCounts);

    void SetStartTime(unsigned int nTime);

    bool GetScanData();

};

} // namespace mapping

using RawMapSingleton = MagicSingleton2<mapping::CRawMap>;

