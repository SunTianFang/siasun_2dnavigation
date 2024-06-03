//
//   The interface of class "CWatchDog".
//

#pragma once

#include <mutex>
#include "MagicSingleton.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CWatchDog".
namespace RK3399 {

class CWatchDog
{
private:
    int fd;
    std::mutex mtx;

private:
    CWatchDog();

    friend MagicSingleton<CWatchDog>;

public:
    bool Init(int ntime);

    bool SetTimeOut(int ntime);

    bool Enable();

    bool Disable();

    bool FeedDog();

    bool Close();

    bool Reset(int ntime);
};

} // namespace RK3399

using WatchDogSingleton = MagicSingleton<RK3399::CWatchDog>;

//auto pA = Singleton<A>::GetInstance();
auto pA = WatchDogSingleton::GetInstance();
