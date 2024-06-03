#pragma once

#ifdef _MSC_VER
#include <windows.h>
#include <time.h>
#else
#include <sys/time.h>
#endif


double getDoubleTime();
