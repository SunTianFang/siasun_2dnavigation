/*
 * LinuxSetting.h
 *
 *  Created on: 2018-6-7
 *      Author: sfe1012
 */

#ifndef LINUXSETTING_H
#define LINUXSETTING_H
#include"PlatformControl.h"


#ifdef DesktopRun
#define  WORK_PATH               "../Parameters/"
#define  LOG_FILE_PATH           "../Parameters/BlackBoxExport/"
#define  LOG_TAR_PATH            "../Parameters/"
#define  FTOK_KEY_PATH           "../Parameters/"
#define  SHAREFILES_PATH         "../Parameters/"
#elif defined _RK3399_ARM_64
#define  WORK_PATH               "/home/proembed/CarryBoy/"
#define  LOG_FILE_PATH           "/home/proembed/CarryBoy/BlackBoxExport/"
#define  LOG_TAR_PATH            "/home/proembed/CarryBoy/"
#define  FTOK_KEY_PATH           "/home/proembed/CarryBoy/"
#elif defined _E3845_LINUX64
#define  WORK_PATH               "/home/siasun/CarryBoy/"
#define  LOG_FILE_PATH           "/home/siasun/CarryBoy/BlackBoxExport/"
#define  LOG_TAR_PATH            "/home/siasun/CarryBoy/"
#define  FTOK_KEY_PATH           "/home/siasun/CarryBoy/"
#elif defined _PiCM4_LINUX32
#define  WORK_PATH               "/home/siasun/CarryBoy/"
#define  LOG_FILE_PATH           "/home/root/CarryBoy/BlackBoxExport/"
#define  LOG_TAR_PATH            "/home/siasun/CarryBoy/"
#define  FTOK_KEY_PATH           "/home/root/CarryBoy/"
#elif defined _RK3588_LINUX64
#define  WORK_PATH               "/userdata/CarryBoy/NAV/"
#define  SHAREFILES_PATH         "/userdata/CarryBoy/ShareFiles/"
#define  LOG_FILE_PATH           "/dev/shm/BlackBoxExport/"
#define  LOG_TAR_PATH            "/userdata/CarryBoy/NAV/"
#define  FTOK_KEY_PATH           "/dev/shm/"
#endif



/*************************NVBlackBox*****************************************/

#define   PLATFORM_WINDOWS_ARMV4I  //open for mrc borax32 nv ram

//FIXME: 暂时导航控制器只能使用一个blackbox,如果两个以上同时使用会产生崩溃,怀疑是共享内存的问题;
//By yu.在同一个控制器中统一导航和车体黑匣子的内存分配
// 临时方案为扩大一个blackbox的容量.
/*
#define  EVEN_BASE_ADDRESS_OFFSET      0
#define  NET_BASE_ADDRESS_OFFSET       512000
#define  CAN_BASE_ADDRESS_OFFSET       1024000           //512000*2
#define  NAV_BASE_ADDRESS_OFFSET       1536000           //512000*3
#define  LASER_BASE_ADDRESS_OFFSET     2048000           //512000*4
#define  CUSTOM_BASE_ADDRESS_OFFSET    2560000           //512000*5
#ifdef _PiCM4_LINUX32
#define  BATTERY_BASE_ADDRESS_OFFSET   3072000           //512000*6
#define  LOC_BASE_ADDRESS_OFFSET       3584000           //512000*7
#define  MSG_BASE_ADDRESS_OFFSET       3584000+512000*10 //512000*17
// 0 - 799:预留为自定义数据交互区
#define  RESERVE_DATA_ADDRESS_OFFSET   9216000           //512000*18
#define  SHARE_MEM_MAX_SIZE            9216800           //9216000+800
#else
#define  LOC_BASE_ADDRESS_OFFSET       3072000           //512000*6
#define  MSG_BASE_ADDRESS_OFFSET       8192000           //3072000+512000*10
#define  LOCAL_PATH_PLANNER_BASE_ADDRESS_OFFSET    8704000     //8192000+512000  size 512000*10       避障
#define  GLOBAL_PATH_PLANER_BASE_ADDRESS_OFFSET    13824000    //8704000+512000*10  size 512000     全局路径规划
#define  CLEAN_PATH_PLANER_BASE_ADDRESS_OFFSET     14336000    //13824000+512000 size 512000           清扫路径规划
#define  CMS_BASE_ADDRESS_OFFSET       14848000               //14336000+512000 size 512000                              cms预留位置
#define  CAMERA_BASE_ADDRESS_OFFSET    15360000               //14848000+512000 size 512000                          相机预留位置
// 0 - 799:预留为自定义数据交互区
#define  RESERVE_DATA_ADDRESS_OFFSET   15872000           //15360000 + 512000  size 800
#define  SHARE_MEM_MAX_SIZE            15872800           //15872000+800

// 0 - 799:预留为自定义数据交互区
#define  RESERVE_DATA_ADDRESS_OFFSET   8704000
#define  SHARE_MEM_MAX_SIZE            8704800           //512000*17+800
*/

//test
#define  EVEN_BASE_ADDRESS_OFFSET      0                 //size 512000
#define  NET_BASE_ADDRESS_OFFSET       512000            //size 512000
#define  CAN_BASE_ADDRESS_OFFSET       1024000           //512000*2  size 512000
#define  NAV_BASE_ADDRESS_OFFSET       1536000           //512000*3  size 512000
#define  LASER_BASE_ADDRESS_OFFSET     2048000           //512000*4  size 512000
#define  CUSTOM_BASE_ADDRESS_OFFSET    2560000           //512000*5  size 512000

#define  LOC_BASE_ADDRESS_OFFSET       3072000           //512000*6  size 512000*10
#define  MSG_BASE_ADDRESS_OFFSET       8192000           //3072000+512000*10 size 512000
#define  LOCAL_PATH_PLANNER_BASE_ADDRESS_OFFSET    8704000     //8192000+512000  size 512000*10       避障
#define  GLOBAL_PATH_PLANER_BASE_ADDRESS_OFFSET    13824000    //8704000+512000*10  size 512000     全局路径规划
#define  CLEAN_PATH_PLANER_BASE_ADDRESS_OFFSET     14336000    //13824000+512000 size 512000           清扫路径规划
#define  CMS_BASE_ADDRESS_OFFSET       14848000               //14336000+512000 size 512000                              cms预留位置
#define  CAMERA_BASE_ADDRESS_OFFSET    15360000               //14848000+512000 size 512000                          相机预留位置
#define  COSTMAP_BASE_ADDRESS_OFFSET    15872000               //15360000 +512000 size 512000                          costmap
// 0 - 799:预留为自定义数据交互区
#define  RESERVE_DATA_ADDRESS_OFFSET   16384000           //15872000 + 512000  size 800
#define  SHARE_MEM_MAX_SIZE            16384800           //16384000 +800


#define BB_EVENT_ID              0
#define BB_NET_ID                1
#define BB_CAN_ID                2
#define BB_NAV_ID                3
#define BB_LASER_ID              4
#define BB_CUSTOM_ID             5

#define BLACK_BOX_SIZE           6

/*****************************************************************/

#endif // LINUXSETTING_H
