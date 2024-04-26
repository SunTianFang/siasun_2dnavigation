#ifndef _DBG_H
#define _DBG_H

#include <stdio.h>
#include <stddef.h>

#define VCIL_DEBUG_LOG_FLAG_VMSG       (0x1 << 0)
#define	VCIL_DEBUG_LOG_FLAG_DEBUG      (0x1 << 1)
#define	VCIL_DEBUG_LOG_FLAG_ERROR      (0x1 << 2)
#define	VCIL_DEBUG_LOG_FLAG_CMD_INFO   (0x1 << 3)

#define VCIL_DEBUG_LOG_FLAG_DEFAULT    VCIL_DEBUG_LOG_FLAG_DEBUG | VCIL_DEBUG_LOG_FLAG_ERROR | VCIL_DEBUG_LOG_FLAG_CMD_INFO | VCIL_DEBUG_LOG_FLAG_VMSG

#ifdef DEBUG
    #ifdef _WIN32_WCE
        #define DEBUG_PRINT(fmt,...) RETAILMSG(TRUE,(TEXT(fmt),__VA_ARGS__))
    #else
        #define DEBUG_PRINT(fmt,...) fprintf(stderr, fmt,## __VA_ARGS__);
    #endif

#else
	#define DEBUG_PRINT(fmt,...) printf(fmt,## __VA_ARGS__);

#endif

#define VCIL_LOG_V(fmt, ... ) if( VCIL_DEBUG_LOG_FLAG_VMSG & debug_level ) DEBUG_PRINT( fmt,## __VA_ARGS__)
#define VCIL_LOG_D(fmt, ... ) if( VCIL_DEBUG_LOG_FLAG_DEBUG & debug_level ) DEBUG_PRINT( fmt,## __VA_ARGS__)
#define VCIL_LOG_E(fmt, ... ) if( VCIL_DEBUG_LOG_FLAG_ERROR & debug_level ) DEBUG_PRINT( fmt,## __VA_ARGS__)
#define VCIL_LOG_C(fmt, ... ) if( VCIL_DEBUG_LOG_FLAG_CMD_INFO & debug_level ) DEBUG_PRINT( fmt,## __VA_ARGS__)

extern unsigned int debug_level;
extern void load_debug_config();
extern void start_background_command_monitor();
extern void stop_background_command_monitor();

#endif
