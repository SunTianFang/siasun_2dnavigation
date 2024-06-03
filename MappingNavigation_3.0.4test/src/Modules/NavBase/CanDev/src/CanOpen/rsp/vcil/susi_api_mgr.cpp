#include "rsp/rsp_vcil/headers/SUSI_IMC.h"
#include "vcil_mgr.h"
#include "can.h"
#include "vmsg.h"

#include <sys/time.h>

#include "DebugLog.h"

#define CHECK_VCIL_INIT_STATUS() \
            if ((vcil1 == NULL)&&(vcil2 == NULL)) \
                return IMC_ERR_SDK_NOT_INIT;

#define RUN_CMD_AND_GET_RESULT(cmd) \
		vcil1->ExecuteCmd(cmd); \
		ret = cmd->result; \
		delete cmd;	
		
#define RUN_CMD_AND_GET_RESULT2(cmd) \
		vcil2->ExecuteCmd(cmd); \
		ret = cmd->result; \
		delete cmd;	

static VCILMgr* vcil1 = NULL;
static VCILMgr* vcil2 = NULL;
static bool vcil_init_once = false;

#if 0
USHORT SUSI_IMC_VCIL_Initialize ()
{
	USHORT ret = 0;
	if( vcil1 != NULL)
		return IMC_ERR_INITIALIZED;
		
	load_debug_config();
		
	vcil1 = new VCILMgr();
	if (vcil1 == NULL)
		return IMC_ERR_OBJ_INIT_FAILED;

	ret = vcil1->init("/dev/ttyS0", 921600);
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_INITIALIZE);

	RUN_CMD_AND_GET_RESULT(cmd);
	
	vcil2 = new VCILMgr();
	if (vcil2 == NULL)
	{
		ret = vcil1->deinit();
		delete vcil1;		
		vcil1 = NULL;
		
		return IMC_ERR_OBJ_INIT_FAILED;
	}

	ret = vcil2->init("/dev/ttyS1", 921600);
	
	cmd = new SUSICommand(SUSI_API_CMD_VCIL_INITIALIZE);
	RUN_CMD_AND_GET_RESULT2(cmd);
	
	start_background_command_monitor();
	
	SUSI_IMC_VCIL_ModuleControl(VCIL_MODE_CAN, VCIL_MODE_CAN);
	
	return IMC_ERR_NO_ERROR;
}

USHORT SUSI_IMC_VCIL_Deinitialize ( void )
{
	USHORT ret = 0;
	
	CHECK_VCIL_INIT_STATUS();
	
	vcil1->deinit();
	delete vcil1;		
	vcil1 = NULL;
	
	if(vcil2 != NULL)
	{
		vcil2->deinit();		
		delete vcil2;
		vcil2 = NULL;
	}
	
	stop_background_command_monitor();

	return IMC_ERR_NO_ERROR;
}
#else
USHORT SUSI_IMC_VCIL_Initialize (IN UCHAR can_bus_number)
{
	USHORT ret = 0;
	static int siSdkDebugCfgFlag = 0;
	static int siSdkInitFlag = 0;
	

	if (((can_bus_number == 1) && (vcil1 != NULL))
			|| ((can_bus_number == 2) && (vcil2 != NULL)))
	{
		return IMC_ERR_INITIALIZED;
	}
	
	/*单进程执行一次,多进程执行多次*/
	if (siSdkInitFlag == 0)
	{
		load_debug_config();
		siSdkInitFlag = 1;
	}

	//printf("\r\n 1SUSI_IMC_VCIL_Initialize can %d", can_bus_number);
	
    if (can_bus_number == 1)
    {
		vcil1 = new VCILMgr();
		if (vcil1 == NULL)
			return IMC_ERR_OBJ_INIT_FAILED;

		ret = vcil1->init("/dev/ttyS0", 921600);
		
		SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_INITIALIZE);

		RUN_CMD_AND_GET_RESULT(cmd);
    }

	if (can_bus_number == 2)
	{		
		vcil2 = new VCILMgr();
		if (vcil2 == NULL)
		{
			ret = vcil1->deinit();
			delete vcil1;		
			vcil1 = NULL;
			
			return IMC_ERR_OBJ_INIT_FAILED;
		}

		ret = vcil2->init("/dev/ttyS1", 921600);
		
		SUSICommand* cmd2 = new SUSICommand(SUSI_API_CMD_VCIL_INITIALIZE);
		RUN_CMD_AND_GET_RESULT2(cmd2);
	}

	/*同一个进程start_background_command_monitor函数执行一次*/
	if (siSdkInitFlag == 0)
	{
		start_background_command_monitor();
		siSdkInitFlag = 1;
	}	
	
	//SUSI_IMC_VCIL_ModuleControl(VCIL_MODE_CAN, VCIL_MODE_CAN);
	if (can_bus_number == 1)
	{
		SUSI_IMC_VCIL_ModuleControl_Port1(VCIL_MODE_CAN);
	}
	else if (can_bus_number == 2)
	{
		SUSI_IMC_VCIL_ModuleControl_Port2(VCIL_MODE_CAN);
	}
	else
	{
	
	}
	
	return IMC_ERR_NO_ERROR;
}

USHORT SUSI_IMC_VCIL_Deinitialize (IN UCHAR can_bus_number)
{
	USHORT ret = 0;
	static int siSdkDeintFlag = 0;
	
	CHECK_VCIL_INIT_STATUS();

	if ((can_bus_number == 1) && (vcil1 != NULL))
	{
		vcil1->deinit();
		delete vcil1;		
		vcil1 = NULL;
	}
	
	if ((can_bus_number == 2) && (vcil2 != NULL))
	{
		vcil2->deinit();		
		delete vcil2;
		vcil2 = NULL;
	}
	
	/*单进程释放一次,多进程每个进程释放一次*/
	if (siSdkDeintFlag == 0)
	{
		stop_background_command_monitor();
		siSdkDeintFlag = 1;
	}
	return IMC_ERR_NO_ERROR;
}
#endif


USHORT SUSI_IMC_VCIL_GetLibVersion ( OUT char *version)
{
	CHECK_VCIL_INIT_STATUS();
	
	if( version == NULL)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	sprintf(version, "%s", VCIL_LIB_VERSION);
	
	return IMC_ERR_NO_ERROR;
}

USHORT SUSI_IMC_VCIL_GetFWVersion ( OUT char * version )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if( version == NULL)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	char fw1_version[8];	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_GET_FIRMWARE_VERSION);
	cmd->input_parameter.push_back((void*)fw1_version);

	RUN_CMD_AND_GET_RESULT(cmd);
	
	char fw2_version[8];
	cmd = new SUSICommand(SUSI_API_CMD_VCIL_GET_FIRMWARE_VERSION);
	cmd->input_parameter.push_back((void*)fw2_version);

	RUN_CMD_AND_GET_RESULT2(cmd);
	
	snprintf(version, VCIL_FIRMWARE_LENGTH ,"%s-%s",fw1_version, fw2_version);

	return ret;
}

USHORT SUSI_IMC_VCIL_ResetModule ( void )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_RESET_MODULE);
	RUN_CMD_AND_GET_RESULT(cmd);
	
	cmd = new SUSICommand(SUSI_API_CMD_VCIL_RESET_MODULE);
	RUN_CMD_AND_GET_RESULT2(cmd);
	return ret;
}

USHORT SUSI_IMC_VCIL_ModuleControl ( IN VCIL_CAN_CHANNEL_MODE port1, IN VCIL_CAN_CHANNEL_MODE port2)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_MODULE_CONTROL);
	cmd->input_parameter.push_back((void*) &port1);

	RUN_CMD_AND_GET_RESULT(cmd);
	
	cmd = new SUSICommand(SUSI_API_CMD_VCIL_MODULE_CONTROL);
	cmd->input_parameter.push_back((void*) &port2);

	RUN_CMD_AND_GET_RESULT2(cmd);
	
	return ret;
}


USHORT SUSI_IMC_VCIL_ModuleControl_Port1 ( IN VCIL_CAN_CHANNEL_MODE port)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_MODULE_CONTROL);
	cmd->input_parameter.push_back((void*) &port);

	RUN_CMD_AND_GET_RESULT(cmd);

	return ret;
}

USHORT SUSI_IMC_VCIL_ModuleControl_Port2 ( IN VCIL_CAN_CHANNEL_MODE port)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_VCIL_MODULE_CONTROL);
	cmd->input_parameter.push_back((void*) &port);

	RUN_CMD_AND_GET_RESULT2(cmd);
	
	return ret;
}

USHORT SUSI_IMC_CAN_GetBitTiming ( IN UINT can_bus_number, OUT CAN_SPEED *bit_rate, OUT CAN_BUS_MODE *mode )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_GET_BIT_TIMING);
	cmd->input_parameter.push_back((void*) bit_rate);
	cmd->input_parameter.push_back((void*) mode);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_CAN_SetBitTiming ( IN UINT can_bus_number, IN CAN_SPEED bit_rate )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_SET_BIT_TIMING);
	cmd->input_parameter.push_back((void*) &bit_rate);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_CAN_SetBitTimingSilence ( IN UINT can_bus_number, IN CAN_SPEED bit_rate )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_SET_BIT_TIMING_SILENCE);
	cmd->input_parameter.push_back((void*) &bit_rate);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
		
	return ret;
}

USHORT SUSI_IMC_CAN_SetMessageMask ( IN UINT can_bus_number, IN PIMC_CAN_MASK_OBJECT mask_object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_SET_MESSAGE_MASK);
	cmd->input_parameter.push_back((void*)mask_object);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_CAN_GetMessageMask ( IN UINT can_bus_number, OUT PIMC_CAN_MASK_OBJECT mask_object)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_GET_MESSAGE_MASK);
	cmd->input_parameter.push_back((void*)mask_object);
	
	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_CAN_RemoveMessageMask ( IN UINT can_bus_number, IN int mask_number )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_REMOVE_MESSAGE_MASK);
	cmd->input_parameter.push_back((void*) &mask_number);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_CAN_ResetMessageMask ( IN UINT can_bus_number )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_RESET_MESSAGE_MASK);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_CAN_GetBusErrorStatus ( IN UINT can_bus_number, OUT PIMC_CAN_ERROR_STATUS_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
				
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_GET_BUS_ERROR_STATUS);	
	cmd->input_parameter.push_back((void*) object);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_CAN_Read ( IN UINT can_bus_number, OUT PIMC_CAN_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
		
	if( can_bus_number == 1)
		ret = vcil1->data_ctx->can_data_ctx.Read(NULL, object);
	else if( can_bus_number == 2)
	{
		ret = vcil2->data_ctx->can_data_ctx.Read(NULL, object);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;

	return ret;
}

USHORT SUSI_IMC_CAN_ReadTimedWait ( IN UINT can_bus_number, OUT PIMC_CAN_MSG_OBJECT object , unsigned int ms)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
		
	if( can_bus_number == 1)
		ret = vcil1->data_ctx->can_data_ctx.ReadTimedWait(object, ms);
	else if( can_bus_number == 2)
	{
		ret = vcil2->data_ctx->can_data_ctx.ReadTimedWait(object, ms);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
		
	if( ret == IMC_ERR_OPERATION_FAILED)
		ret = IMC_CAN_COMMAND_RECV_ERROR;

	return ret;
}

USHORT SUSI_IMC_CAN_Write ( IN UINT can_bus_number, IN PIMC_CAN_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_WRITE);
	cmd->input_parameter.push_back((void*)object);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if(can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;

	return ret;
}

USHORT SUSI_IMC_CAN_SetEvent ( IN UINT can_bus_number, void* hEvent )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if (hEvent == NULL)
		return IMC_ERR_INVALID_ARGUMENT;

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_CAN_SET_EVENT);
	cmd->input_parameter.push_back((void*)hEvent);

	if( can_bus_number == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( can_bus_number == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_J1939_Read ( IN UINT port, OUT PIMC_J1939_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if( port == 1)
		ret = vcil1->data_ctx->j1939_data_ctx.Read(object);
	else if( port == 2)
		ret = vcil2->data_ctx->j1939_data_ctx.Read(object);
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_J1939_ReadTimedWait ( IN UINT port, OUT PIMC_J1939_MSG_OBJECT object, unsigned int ms )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if( port == 1)
		ret = vcil1->data_ctx->j1939_data_ctx.ReadTimedWait(object, ms);
	else if( port == 2)
		ret = vcil2->data_ctx->j1939_data_ctx.ReadTimedWait(object, ms);
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_J1939_Write ( IN UINT port, IN PIMC_J1939_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_WRITE);
	cmd->input_parameter.push_back((void*)object);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;

	return ret;
}

USHORT SUSI_IMC_J1939_SetEvent( IN UINT port, void* hEvent )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if (hEvent == NULL)
		return IMC_ERR_INVALID_ARGUMENT;

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_SET_EVENT);
	cmd->input_parameter.push_back((void*)hEvent);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_J1939_AddMessageFilter(IN UINT port, UINT pgn )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_ADD_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) &pgn);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_J1939_GetMessageFilter(IN UINT port, OUT UINT* total, UINT* pgn )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_GET_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) total);
	cmd->input_parameter.push_back((void*) pgn);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;	
	return ret;
}

USHORT SUSI_IMC_J1939_RemoveMessageFilter(IN UINT port, UINT pgn )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_REMOVE_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) &pgn);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_J1939_RemoveAllMessageFilter ( IN UINT port )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_REMOVE_ALL_MESSAGE_FILTER);
	
	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_J1939_SetTransmitConfig(IN UINT port, IN IMC_J1939_TRANSMIT_CONFIG transmit_config)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_SET_ADDRESS_AND_NAME);
    cmd->input_parameter.push_back((void*) &transmit_config);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}
USHORT SUSI_IMC_J1939_GetTransmitConfig(IN UINT port, OUT PIMC_J1939_TRANSMIT_CONFIG ptransmit_config)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_J1939_GET_ADDRESS_AND_NAME);
    cmd->input_parameter.push_back((void*) ptransmit_config);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	
	return ret;
}

USHORT SUSI_IMC_OBD2_Read(IN UINT port, OUT PIMC_OBD2_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if( port == 1)
 	   ret = vcil1->data_ctx->obd2_data_ctx.Read(object);
 	else if( port == 2)
 		ret = vcil2->data_ctx->obd2_data_ctx.Read(object);
 	else
		return IMC_ERR_INVALID_ARGUMENT; 	
    
	return ret;
}

USHORT SUSI_IMC_OBD2_ReadTimedWait(IN UINT port, OUT PIMC_OBD2_MSG_OBJECT object, unsigned int ms)
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if( port == 1)
 	   ret = vcil1->data_ctx->obd2_data_ctx.ReadTimedWait(object, ms);
 	else if( port == 2)
 		ret = vcil2->data_ctx->obd2_data_ctx.ReadTimedWait(object, ms);
 	else
		return IMC_ERR_INVALID_ARGUMENT; 	
    
	return ret;
}

USHORT SUSI_IMC_OBD2_Write(IN UINT port, IN PIMC_OBD2_MSG_OBJECT object )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_WRITE);

    cmd->input_parameter.push_back((void*) object);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_OBD2_SetEvent(IN UINT port, void* hEvent )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	if (hEvent == NULL)
		return IMC_ERR_INVALID_ARGUMENT;

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_SET_EVENT);
	cmd->input_parameter.push_back((void*)hEvent);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_OBD2_AddMessageFilter(IN UINT port, UINT pid )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_ADD_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) &pid);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_OBD2_GetMessageFilter(IN UINT port, UINT* total, UINT* pid )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
		
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_GET_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) total);
	cmd->input_parameter.push_back((void*) pid);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

USHORT SUSI_IMC_OBD2_RemoveMessageFilter( IN UINT port, IN UINT pid )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();
	
	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_REMOVE_MESSAGE_FILTER);
	cmd->input_parameter.push_back((void*) &pid);

	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}
USHORT SUSI_IMC_OBD2_RemoveAllMessageFilter ( IN UINT port )
{
	USHORT ret = 0;
	CHECK_VCIL_INIT_STATUS();

	SUSICommand* cmd = new SUSICommand(SUSI_API_CMD_OBD2_REMOVE_ALL_MESSAGE_FILTER);
	
	if( port == 1)
	{
		RUN_CMD_AND_GET_RESULT(cmd);
	}
	else if( port == 2)
	{
		RUN_CMD_AND_GET_RESULT2(cmd);
	}
	else
		return IMC_ERR_INVALID_ARGUMENT;
	return ret;
}

