#ifndef _J1939_H
#define _J1939_H

#include "rsp/rsp_vcil/headers/SUSI_IMC.h"
#include "vcil_ctx.h"

class J1939Mgr : public VehicleCommunicationMessageContent
{
public:
	J1939Mgr();
	virtual ~J1939Mgr();

	USHORT Read( OUT PIMC_J1939_MSG_OBJECT object );
	USHORT ReadTimedWait( OUT PIMC_J1939_MSG_OBJECT object , unsigned int ms);
	USHORT Write(VMSG *v, IN PIMC_J1939_MSG_OBJECT object );
	USHORT SetEvent( void* hEvent );
	USHORT AddMessageFilter(VMSG *v, UINT pgn );
	USHORT GetMessageFilter(VMSG *v );
	USHORT RemoveMessageFilter(VMSG *v, UINT pgn );
	USHORT RemoveAllMessageFilter (VMSG *v );

	USHORT SetTransmitConfig(VMSG *v, IN IMC_J1939_TRANSMIT_CONFIG);
	USHORT GetTransmitConfig(VMSG *v, OUT PIMC_J1939_TRANSMIT_CONFIG);

	USHORT total_recv_valid_can_msgs;

	/* FIFO buffer */
	CircularBuffer<IMC_J1939_MSG_OBJECT> buffer;
};

#endif
