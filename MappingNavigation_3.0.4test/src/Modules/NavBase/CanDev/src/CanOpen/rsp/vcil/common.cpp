#include "rsp/rsp_vcil/headers/SUSI_IMC_Types.h"
#include "common.h"
#include "DebugLog.h"

#include <time.h>
#include <string.h>

double GetTickCount(void) 
{
 struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}

void __put_unaligned_2_be ( unsigned long __v, unsigned char *__p )
{
	*__p++ = ( unsigned char ) ( __v >> 8 );
	*__p++ = ( unsigned char ) __v;
}

void __put_unaligned_4_be ( unsigned long __v, unsigned char *__p )
{
	__put_unaligned_2_be ( __v >> 16, __p );
	__put_unaligned_2_be ( __v, __p + 2 );
}

void init_WRPACKET ( RPACKET *x, unsigned char cmd, unsigned char *payload, unsigned payload_length )
{
	memset ( x, 0, sizeof ( RPACKET ) );

	x->sync = SYNC;
	x->cmd = cmd;
	x->payload = payload;
	x->payload_length = payload_length;
	x->length = 1 + x->payload_length + 1;			// 	Summation of CMD, PAYLOAD and checksum
}

void init_RRPACKET ( RPACKET *x, unsigned char *payload, unsigned payload_length )
{
	memset ( x, 0, sizeof ( RPACKET ) );

	x->payload = payload;
	x->payload_length = payload_length;
}

