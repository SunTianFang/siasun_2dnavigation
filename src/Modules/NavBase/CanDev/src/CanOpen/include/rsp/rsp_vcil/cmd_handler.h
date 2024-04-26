#ifndef _CMD_HANDLER_H
#define _CMD_HANDLER_H

#include "SUSI_IMC.h"

#include "vcil_protocol_mgr.h"
#include "vcil_cmd.h"
#include "vcil_vmsg.h"

class SUSICommandHandler
{
public:
	SUSICommandHandler(VehicleCommunicationMessage *vcm, VMSG *v);
	~SUSICommandHandler();	

	SUSICommand *NowRunCmd;

	void AttachCmdAndExcute(SUSICommand* cmd);
	void DetachCmd();
	void CmdDoneNotify(USHORT ret);
	void PollCAN();
protected:
	VehicleCommunicationMessage *protocol_mgr;

private:
	void cmd_handler_func_vcil(SUSICommand* cmd);
	void cmd_handler_func_can(SUSICommand* cmd);
	void cmd_handler_func_j1708(SUSICommand* cmd);
	void cmd_handler_func_j1939(SUSICommand* cmd);
	void cmd_handler_func_j1587(SUSICommand* cmd);
	void cmd_handler_func_obd2(SUSICommand* cmd);
	
	VMSG *vmsg;
};
#endif

