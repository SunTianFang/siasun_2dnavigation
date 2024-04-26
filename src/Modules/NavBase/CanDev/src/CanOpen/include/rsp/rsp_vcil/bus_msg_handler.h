#ifndef _BUS_MSG_HANDLER_H
#define _BUS_MSG_HANDLER_H

#include "SUSI_IMC.h"
#include "vcil_protocol_mgr.h"
#include "cmd_handler.h"

class BusMessageHandler
{
public:
	BusMessageHandler(VehicleCommunicationMessage *vcm, SUSICommandHandler *c, VCILPort *p);
	~BusMessageHandler();	

protected:
	VehicleCommunicationMessage* protocol_mgr;
	SUSICommandHandler *cmder;

	bool enable;
	void StopThread();
private:
	pthread_t ThreadHandle;
	VCILPort *port;
	
	static void* BusMessageHandleThread( void* lpParam );
};

#endif


