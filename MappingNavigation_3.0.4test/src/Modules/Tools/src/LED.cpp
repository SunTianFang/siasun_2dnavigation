//
//   Implementation of class "CLED".
//

#include "RK3399/LED.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CLED".

namespace RK3399 {

CLED::CLED()
{
    m_GreenLed.Set(158, 1);

}

bool CLED::Init()
{

    return 0;
}

bool CLED::SetMode(int nMode)
{

    return 0;
}

} // namespace RK3399

