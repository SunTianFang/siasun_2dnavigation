#ifndef _J1587_H
#define _J1587_H

#include "SUSI_IMC.h"
#include "vcil_ctx.h"

class J1587Mgr : public VehicleCommunicationMessageContent
{
public:
	J1587Mgr();
	virtual ~J1587Mgr();

	USHORT Read( OUT PIMC_J1587_MSG_OBJECT object );
	USHORT Write( IN PIMC_J1587_MSG_OBJECT object );
	USHORT SetEvent( void* hEvent );
	USHORT AddMessageFilter( IN UINT mid );
	USHORT GetMessageFilter( void );
	USHORT RemoveMessageFilter( IN UINT mid );
	USHORT RemoveAllMessageFilter ( void );

	ULONG total_recv_can_msgs;

	/* FIFO buffer */
	CircularBuffer<IMC_J1587_MSG_OBJECT> buffer;
};

#endif
