//
//   Implementation of class "CGpio".
//

#include <stdio.h>
#include "RK3399/Gpio.h"

#if __cplusplus >= 201103L
#endif //__cplusplus

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CGpio".

namespace RK3399 {

CGpio::CGpio()
{
    m_pin = 0;
    m_dir = 0;
}

void CGpio::Set(int pin, int dir)
{
    m_pin = pin;
    m_dir = dir;
}

int CGpio::Export()
{
    FILE *fp = NULL;

    std::lock_guard<std::mutex> lock(mtx);
    fp = fopen("/sys/class/gpio/export","w");
    if(fp == NULL)
    {
        return -1;
    }

    fprintf(fp,"%d", m_pin);
    fclose(fp);

    return 0;
}

int CGpio::Unexport()
{
    FILE *fp = NULL;

    std::lock_guard<std::mutex> lock(mtx);
    fp = fopen("/sys/class/gpio/unexport","w");
    if(fp == NULL)
    {
        return -1;
    }

    fprintf(fp,"%d",m_pin);
    fclose(fp);

    return 0;
}

int CGpio::Direction()
{
    FILE *fp = NULL;
    char str[50] = "";

    sprintf(str,"/sys/class/gpio/gpio%d/direction", m_pin);

    std::lock_guard<std::mutex> lock(mtx);
    fp = fopen(str,"w");
    if(fp == NULL)
    {
        return -1;
    }

    if(m_dir == 1) {
        fprintf(fp,"out");
    }
    else if (m_dir == 0) {
        fprintf(fp,"in");
    }

    fclose(fp);

    return 0;
}

int CGpio::Write(int value)
{
    FILE *fp = NULL;
    char str[50] = "";

    sprintf(str,"/sys/class/gpio/gpio%d/value", m_pin);

    std::lock_guard<std::mutex> lock(mtx);
    fp = fopen(str,"w");
    if(fp == NULL)
    {
        return -1;
    }

    fprintf(fp,"%d", value);
    fclose(fp);

    return 0;
}

int CGpio::Read()
{
    return 0;
}

} // namespace RK3399

