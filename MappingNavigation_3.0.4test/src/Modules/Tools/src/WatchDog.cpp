//
//   Implementation of class "CWatchDog".
//

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include "RK3399/WatchDog.h"

#if __cplusplus >= 201103L
#endif //__cplusplus

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CWatchDog".

namespace RK3399 {

CWatchDog::CWatchDog()
{
    fd = -1;
}

bool CWatchDog::Init(int ntime)
{
    fd = open("/dev/watchdog", O_WRONLY);

    if (fd < 0)
    {
        printf("open wdt err!");
        return false;
    }

    // set time out
    if(!SetTimeOut(ntime))
    {
        return false;
    }

    // disable
    if(!Disable())
    {
        return false;
    }

    return true;
}

bool CWatchDog::SetTimeOut(int ntime)
{
    if(fd < 0)
    {
        return false;
    }

    int ntime_ = ntime;
    if(ntime_ <= 0)
    {
        ntime_ = 3;
    }

    std::lock_guard<std::mutex> lock(mtx);
    auto ret = ioctl(fd, WDIOC_SETTIMEOUT, &ntime_);
    return (ret == 0);
}

bool CWatchDog::Enable()
{
    if(fd < 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx);
    auto ret = ioctl(fd, WDIOC_SETOPTIONS, WDIOS_ENABLECARD);
    return (ret == 0);
}

bool CWatchDog::Disable()
{
    if(fd < 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx);
    auto ret = ioctl(fd, WDIOC_SETOPTIONS, WDIOS_DISABLECARD);
    return (ret == 0);
}

bool CWatchDog::FeedDog()
{
    if(fd < 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx);
    auto ret = ioctl(fd, WDIOC_KEEPALIVE, NULL);
    return (ret == 0);
}

bool CWatchDog::Close()
{
    Disable();

    if(fd > 0)
    {
        close(fd);
        fd = -1;
    }

    return true;
}

bool CWatchDog::Reset(int ntime)
{
    Close();
    auto ret = Init(ntime);

    return ret;
}

} // namespace RK3399

