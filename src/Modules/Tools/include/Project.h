//                     - PROJECT.H -
//
//   根据项目的需求进行配置
//
#ifndef __PROJECT
#define __PROJECT
#include"LinuxSetting.h"

#define VER_IN_BLACKBOX		"3.1.1.3_b"
#define AGV_SOFTWARE_VER          (VER_IN_BLACKBOX)
#define DATE_OF_RELEASE           (__DATE__)
#define TIME_OF_RELEASE           (__TIME__)


#define  USE_LEG_METHOD

#ifdef _RK3399_ARM_64   //Change By yu. Fix [Mapping] can't run after add blockbox.
#define USE_BLACK_BOX
#elif defined _E3845_LINUX64
#define USE_BLACK_BOX
#elif defined _PiCM4_LINUX32
    #define USE_BLACK_BOX
#elif defined _RK3588_LINUX64
    #define USE_BLACK_BOX
#endif


#endif
