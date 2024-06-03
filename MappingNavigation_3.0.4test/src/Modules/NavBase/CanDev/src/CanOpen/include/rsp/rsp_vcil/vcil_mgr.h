#ifndef _VCIL_MGR_H
#define _VCIL_MGR_H

#include "SUSI_IMC.h"
#include "cmd_handler.h"
#include "bus_msg_handler.h"
#include "vcil_vmsg.h"

class VCILMgr
{
public:
	VCILMgr();
	~VCILMgr();

	VehicleCommunicationMessage* data_ctx;	
	SUSICommandHandler* cmd_handler;
	BusMessageHandler* bus_message_handler;

	USHORT init(const char* can_port, int init_baudrate);
	USHORT deinit();

	USHORT SendCmd(SUSICommand* cmd);
	USHORT GetResult(SUSICommand* cmd);
	USHORT ExecuteCmd(SUSICommand* cmd);
	
	int id;
	
private:
	pthread_mutex_t cmd_mutex;
	
	VCILPort port;
	VMSG *vmsg;
	static int instance_number;
};

#endif

