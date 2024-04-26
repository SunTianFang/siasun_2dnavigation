#ifndef _OBD2_H
#define _OBD2_H

#include "SUSI_IMC.h"
#include "vcil_ctx.h"

class OBD2Mgr : public VehicleCommunicationMessageContent
{
public:
	OBD2Mgr();
	virtual ~OBD2Mgr();

	USHORT Read( OUT PIMC_OBD2_MSG_OBJECT object );
	USHORT ReadTimedWait( OUT PIMC_OBD2_MSG_OBJECT object , unsigned int ms);
	USHORT Write(VMSG *v, IN PIMC_OBD2_MSG_OBJECT object );

	USHORT SetEvent( void* hEvent );
	USHORT AddMessageFilter(VMSG *v, UINT pid );
	USHORT GetMessageFilter(VMSG *v );
	USHORT RemoveMessageFilter(VMSG *v, UINT pid );
	USHORT RemoveAllMessageFilter (VMSG *v);

    USHORT total_recv_can_frames;

	/* FIFO buffer */
	CircularBuffer<IMC_OBD2_MSG_OBJECT> buffer;
};

#endif
