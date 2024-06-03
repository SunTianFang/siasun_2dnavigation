//
//   The interface of class "CSensorFamily".
//

#pragma once

#include <vector>
#include <mutex>
#include <atomic>
#include "MagicSingleton.h"
#include "RangeScanner.h"
#include "PfR2000LaserScanner.h"
#include "HokuyoLaserScanner.h"
#include "SickSafetyLaserScanner.h"
#include "ScannerParam.h"
#include "Sick581LaserScanner.h"
#include "ThreadHelper.h"
#include "CPing.h"
#include "WJ716LaserScanner.h"
#include "WJ719LaserScanner.h"
#include "FRR2LaserScanner.h"
#include "LeimouF30LaserScanner.h"
#include "BlueSeaLaser50c.h"
#include "BlueSeaLaserE320.h"


namespace sensor {

class CSensorData
{
public:
    int id;
    CRangeScanner* scanner;
    CLaserScannerParam* parm;

public:
    CSensorData()
    {
        id = 0;
        scanner = NULL;
        parm = NULL;
    }

    ~CSensorData()
    {
        Clear();
    }

    void Clear()
    {
        if (scanner != NULL) {
            delete scanner;
            scanner = NULL;
        }

        if (parm != NULL) {
            delete parm;
            parm = NULL;
        }
    }
};

class CSensorFamily : public CThreadHelper
{
private:
    std::mutex sensor_mtx;
    std::atomic<int> m_aCount;
    std::vector<CSensorData*> sensor_family;
    bool m_bStarted;

    CPing m_Ping;

private:
    CSensorFamily();

    void Clear();

    bool LoadLaserParam();

    friend MagicSingleton<CSensorFamily>;

protected:
    virtual void SupportRoutineProxy();

public:
    ~CSensorFamily();

    bool Initialize();

    CScannerGroupParam GetScannerGroupParam();

    unsigned int GetCount();

    bool GetState(unsigned int index);

    bool DataReady(unsigned int index);

    int GetType(unsigned int index);

    bool Stop();

    bool GetPointCloud(unsigned int index,int*& pDist, int*& pIntensity);

    bool GetRawPointCloud(unsigned int index, std::shared_ptr<sensor::CRawPointCloud>& pCloud);

    CSensorData* GetSensorData(unsigned int index);

    bool IsBlocked();
};

} // namespace sensor

using SensorFamilySingleton = MagicSingleton<sensor::CSensorFamily>;

//using SensorFamilySingleton = MagicSingleton<sensor::CSensorFamily>;
