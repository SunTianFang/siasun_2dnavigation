#ifndef _CAN_H
#define _CAN_H

#include "SUSI_IMC.h"
#include "vcil_ctx.h"

#define CAN_BUF_RX_SIZE   1024

class CanMgr : public VehicleCommunicationMessageContent
{
public:
	CanMgr();
	virtual ~CanMgr();

	/* API  */
	USHORT Read(VMSG *v, PIMC_CAN_MSG_OBJECT object);
	USHORT ReadTimedWait(PIMC_CAN_MSG_OBJECT object, unsigned int ms);
	USHORT Write (VMSG *v, IN PIMC_CAN_MSG_OBJECT object );

	USHORT SetEvent ( void* hEvent );
	USHORT SetBitTiming (VMSG *v, IN CAN_SPEED bit_rate, IN int silence_enable );
	USHORT GetBitTiming (VMSG *v);

	USHORT SetMessageMask (VMSG *v, IN PIMC_CAN_MASK_OBJECT mask_object );
	USHORT GetMessageMask (VMSG *v, OUT PIMC_CAN_MASK_OBJECT mask_object);
	USHORT RemoveMessageMask (VMSG *v, IN int mask_number );
	USHORT ResetMessageMask (VMSG *v );
	USHORT GetErrorStatus(VMSG *v );

	/* FIFO buffer */
	CircularBuffer<IMC_CAN_MSG_OBJECT> buffer;
	int tx_type; // 0 old 1 new
};

#endif
