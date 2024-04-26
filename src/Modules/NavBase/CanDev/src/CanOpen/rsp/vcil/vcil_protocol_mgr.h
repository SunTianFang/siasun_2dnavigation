#ifndef _VCIL_PROTOCOL_MGR_H
#define _VCIL_PROTOCOL_MGR_H

#include "can.h"
#include "j1939.h"
#include "obd2.h"

class VehicleCommunicationMessage
{
public:
	CanMgr can_data_ctx;
	J1939Mgr j1939_data_ctx;
	OBD2Mgr obd2_data_ctx;
};

#endif
