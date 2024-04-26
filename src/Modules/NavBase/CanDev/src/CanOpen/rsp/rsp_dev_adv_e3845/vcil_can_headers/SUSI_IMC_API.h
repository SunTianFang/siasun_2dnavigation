
#ifndef _SUSI_IMC_API_H
#define _SUSI_IMC_API_H

#ifndef DLLAPI
#if defined (WIN32) || defined (WINCE)
#ifdef IMC_EXPORTS
#define DLLAPI __declspec( dllexport )
#else
#define DLLAPI __declspec( dllimport )
#endif
	#elif defined (__linux__) || defined (ANDROID)
#define DLLAPI
#else
#error:Unknown OS
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#include "SUSI_IMC_VCIL_API.h"
#include "SUSI_IMC_CAN_API.h"
#include "SUSI_IMC_J1939_API.h"
#include "SUSI_IMC_OBD2_API.h"

#ifdef __cplusplus
}
#endif

#endif
