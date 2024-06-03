#ifndef _J1708_H
#define _J1708_H

#include "SUSI_IMC.h"
#include "vcil_ctx.h"

class J1708Mgr : public VehicleCommunicationMessageContent
{
public:
	J1708Mgr();
	virtual ~J1708Mgr();
	
	/* API  */
	USHORT Read( OUT PIMC_J1708_MSG_OBJECT object );
	USHORT Write( IN PIMC_J1708_MSG_OBJECT object );

	USHORT SetEvent( void* hEvent );
	USHORT AddMessageFilter( IN UINT mid );
	USHORT GetMessageFilter( void );
	USHORT RemoveMessageFilter( IN UINT mid );
	USHORT RemoveAllMessageFilter ( void );

	/* FIFO buffer */
	CircularBuffer<IMC_J1708_MSG_OBJECT> buffer;	
};

#endif
