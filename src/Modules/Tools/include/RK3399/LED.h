//
//   The interface of class "CLED".
//

#pragma once

#include "RK3399/Gpio.h"
#include "MagicSingleton.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLED".
namespace RK3399 {

class CLED
{
private:
    CGpio m_GreenLed;

private:
    CLED();

    friend MagicSingleton<CLED>;

public:
    bool Init();

    bool SetMode(int nMode);

};

} // namespace RK3399

using LEDSingleton = MagicSingleton<RK3399::CLED>;

