#include <string.h>
#include "cmd_handler.h"
#include "vmsg.h"
#include "DebugLog.h"
#include "vcil_port.h"

 #include <unistd.h>

#define CHECK_RESULT( ret ) \
        { \
            if (IMC_ERR_NO_ERROR != ret) \
                cmd->Done( ret ); \
        }

SUSICommandHandler::SUSICommandHandler(VehicleCommunicationMessage *vcm, VMSG *v)
:NowRunCmd(NULL)
{
	protocol_mgr = vcm;
	vmsg = v;
}

SUSICommandHandler::~SUSICommandHandler()
{
}

void SUSICommandHandler::CmdDoneNotify(USHORT ret)
{
	if(NowRunCmd != NULL)
		NowRunCmd->Done(ret);
}

void SUSICommandHandler::DetachCmd()
{
	NowRunCmd = NULL;
}

void SUSICommandHandler::PollCAN()
{
	vmsg->vmsg_poll_can ();
}

void SUSICommandHandler::AttachCmdAndExcute(SUSICommand* cmd)
{
	NowRunCmd = cmd;
	
	switch(cmd->cmd_group)
	{
		case CMD_GROUP_VCIL:
			cmd_handler_func_vcil(cmd);
			break;
		case CMD_GROUP_CAN:
			cmd_handler_func_can(cmd);
			break;
		case CMD_GROUP_J1939:
			cmd_handler_func_j1939(cmd);
			break;
		case CMD_GROUP_OBD2:
			cmd_handler_func_obd2(cmd);
			break;
		default:
			VCIL_LOG_E("unnkown command cmd group %d\n", cmd->cmd_group);
			cmd->Done( IMC_ERR_UNKNOWN );
			break;
	}
}

void SUSICommandHandler::cmd_handler_func_vcil( SUSICommand* cmd )
{
	//VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil ++  \r\n");

	switch (cmd->susi_cmd)
	{
		case SUSI_API_CMD_VCIL_INITIALIZE:
		{
			VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil - SUSI_API_CMD_VCIL_INITIALIZE  \r\n");
			
            vmsg->vmsg_firmware_version();
			break;
		}
		case SUSI_API_CMD_VCIL_GET_FIRMWARE_VERSION:
		{
			VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil - SUSI_API_CMD_VCIL_GET_FIRMWARE_VERSION  \r\n");

			vmsg->vmsg_firmware_version();
			break;
		}
		case SUSI_API_CMD_VCIL_GET_LIB_VERSION:
		{
			VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil - SUSI_API_CMD_VCIL_GET_LIB_VERSION  \r\n");

			PBYTE version = (PBYTE) cmd->input_parameter.front();
			memcpy( version, VCIL_LIB_VERSION, sizeof(char) * strlen(VCIL_LIB_VERSION));

			cmd->Done( IMC_ERR_NO_ERROR );

			break;
		}
		case SUSI_API_CMD_VCIL_RESET_MODULE:
		{
			VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil - SUSI_API_CMD_VCIL_RESET_MODULE  \r\n");
			
			vmsg->vmsg_reset_module();
			
			cmd->Done( IMC_ERR_NO_ERROR );
			break;
		}	
		case SUSI_API_CMD_VCIL_MODULE_CONTROL:
		{
			VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil - SUSI_API_CMD_VCIL_MODULE_CONTROL  \r\n");

			unsigned char parm_mode = 0;

			VCIL_CAN_CHANNEL_MODE can_channel_1 = *(PVCIL_CAN_CHANNEL_MODE) cmd->input_parameter.front();cmd->input_parameter.pop_front();

			if (( can_channel_1 < VCIL_MODE_CAN) || ( can_channel_1 > VCIL_MODE_OBD2) )
			{
				cmd->Done( IMC_ERR_INVALID_ARGUMENT );
				break;
			}
			
			unsigned char protocol = 0x00;
			
			switch(can_channel_1)
			{
				case VCIL_MODE_CAN:
					protocol = 0x00;				
					break;
				case VCIL_MODE_J1939:
					protocol = 0x01;				
					break;
				case VCIL_MODE_OBD2:
					protocol = 0x2;				
					break;
				default:
					cmd->Done( IMC_ERR_INVALID_ARGUMENT );
					return;
			}
		
			vmsg->vmsg_module_control( protocol );	
			cmd->Done( IMC_ERR_NO_ERROR );

			break;
		}
		default:
		break;
	}

	//VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_vcil -- \r\n");
}

void SUSICommandHandler::cmd_handler_func_can( SUSICommand* cmd )
{
	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_can ++  \r\n");

	switch (cmd->susi_cmd)
	{
		case SUSI_API_CMD_CAN_SET_BIT_TIMING:
		{
			CAN_SPEED *bit_rate = (CAN_SPEED*) cmd->input_parameter.front();

			USHORT result = protocol_mgr->can_data_ctx.SetBitTiming(vmsg, *bit_rate, 0);

            cmd->Done( result );

			break;
		}
		case SUSI_API_CMD_CAN_READ:
		{
			PIMC_CAN_MSG_OBJECT object = (PIMC_CAN_MSG_OBJECT) cmd->input_parameter.front();
			
			cmd->Done( protocol_mgr->can_data_ctx.Read(vmsg, object));
			break;
		}
		case SUSI_API_CMD_CAN_SET_BIT_TIMING_SILENCE:
		{
			CAN_SPEED *bit_rate = (CAN_SPEED*) cmd->input_parameter.front();

			USHORT result = protocol_mgr->can_data_ctx.SetBitTiming(vmsg, *bit_rate, 1);

            cmd->Done( result );
			break;
		}
		case SUSI_API_CMD_CAN_SET_MESSAGE_MASK:
		{
			PIMC_CAN_MASK_OBJECT mask_object = (PIMC_CAN_MASK_OBJECT) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->can_data_ctx.SetMessageMask(vmsg, mask_object) );
			break;
		}
		case SUSI_API_CMD_CAN_GET_MESSAGE_MASK:
		{
			PIMC_CAN_MASK_OBJECT mask_object = (PIMC_CAN_MASK_OBJECT) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->can_data_ctx.GetMessageMask(vmsg, mask_object) );
			break;
		}
		case SUSI_API_CMD_CAN_REMOVE_MESSAGE_MASK:
		{
			int *mask_number = (int*) cmd->input_parameter.front();

			CHECK_RESULT(protocol_mgr->can_data_ctx.RemoveMessageMask(vmsg, *mask_number));
			break;
		}
		case SUSI_API_CMD_CAN_RESET_MESSAGE_MASK:
		{
			CHECK_RESULT( protocol_mgr->can_data_ctx.ResetMessageMask(vmsg) );
			break;
		}
		case SUSI_API_CMD_CAN_GET_BUS_ERROR_STATUS:
		{
			CHECK_RESULT( protocol_mgr->can_data_ctx.GetErrorStatus(vmsg) );
			break;
		}
		case SUSI_API_CMD_CAN_GET_BIT_TIMING:
		{
			CHECK_RESULT( protocol_mgr->can_data_ctx.GetBitTiming(vmsg) );
			break;
		}
		case SUSI_API_CMD_CAN_WRITE:
		{			
			PIMC_CAN_MSG_OBJECT object = (PIMC_CAN_MSG_OBJECT) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->can_data_ctx.Write(vmsg, object) );			
			//cmd->Done( protocol_mgr->can_data_ctx.Write(vmsg, object));
			break;
		}
		case SUSI_API_CMD_CAN_SET_EVENT:
		{
			cmd->Done( protocol_mgr->can_data_ctx.SetEvent( (HANDLE*) cmd->input_parameter.front() ) );
			break;
		}
		default:
			break;
	}

	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_can --  \r\n");
}

void SUSICommandHandler::cmd_handler_func_j1939(SUSICommand* cmd)
{
	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_j1939 ++  \r\n");

	switch (cmd->susi_cmd)
	{
		case SUSI_API_CMD_J1939_READ:
		{
			/* Not handle in this*/
			break;
		}
		case SUSI_API_CMD_J1939_WRITE:
		{
			PIMC_J1939_MSG_OBJECT object = (PIMC_J1939_MSG_OBJECT) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->j1939_data_ctx.Write(vmsg, object) )
			break;
		}
		case SUSI_API_CMD_J1939_SET_EVENT:
		{
			cmd->Done( protocol_mgr->j1939_data_ctx.SetEvent( (HANDLE*) cmd->input_parameter.front() ) );
			break;
		}	
		case SUSI_API_CMD_J1939_ADD_MESSAGE_FILTER:
		{
			UINT pgn = *(UINT*) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->j1939_data_ctx.AddMessageFilter(vmsg, pgn ) )
			break;
		}
		case SUSI_API_CMD_J1939_GET_MESSAGE_FILTER:
		{
			CHECK_RESULT( protocol_mgr->j1939_data_ctx.GetMessageFilter(vmsg) )
			break;
		}
		case SUSI_API_CMD_J1939_REMOVE_MESSAGE_FILTER:
		{
			UINT pgn = *(UINT*) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->j1939_data_ctx.RemoveMessageFilter(vmsg, pgn ) )
			break;
		}
		case SUSI_API_CMD_J1939_REMOVE_ALL_MESSAGE_FILTER:
		{
			CHECK_RESULT( protocol_mgr->j1939_data_ctx.RemoveAllMessageFilter(vmsg) )
			break;
		}
		case SUSI_API_CMD_J1939_SET_ADDRESS_AND_NAME:
		{
			IMC_J1939_TRANSMIT_CONFIG trainsmit_config = *(IMC_J1939_TRANSMIT_CONFIG*) cmd->input_parameter.front(); cmd->input_parameter.pop_front();
			CHECK_RESULT( protocol_mgr->j1939_data_ctx.SetTransmitConfig(vmsg, trainsmit_config) )
			break;
		}
		case SUSI_API_CMD_J1939_GET_ADDRESS_AND_NAME:
		{            
 			PIMC_J1939_TRANSMIT_CONFIG ptrainsmit_config = (PIMC_J1939_TRANSMIT_CONFIG) cmd->input_parameter.front();
			CHECK_RESULT( protocol_mgr->j1939_data_ctx.GetTransmitConfig(vmsg, ptrainsmit_config) )
			break;
		}
		default:
			break;
	}

	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_j1939 --  \r\n");
}

void SUSICommandHandler::cmd_handler_func_obd2(SUSICommand* cmd)
{
	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_obd2 ++  \r\n");

	switch (cmd->susi_cmd)
	{
		case SUSI_API_CMD_OBD2_READ:
		{
			/* Not handle in this*/
			break;
		}
		case SUSI_API_CMD_OBD2_WRITE:
		{
			PIMC_OBD2_MSG_OBJECT object = (PIMC_OBD2_MSG_OBJECT) cmd->input_parameter.front();
			
			CHECK_RESULT( protocol_mgr->obd2_data_ctx.Write(vmsg, object) )
			break;
		}
		case SUSI_API_CMD_OBD2_SET_EVENT:
		{
			cmd->Done( protocol_mgr->obd2_data_ctx.SetEvent((HANDLE*) cmd->input_parameter.front() ) );
			break;
		}	
		case SUSI_API_CMD_OBD2_ADD_MESSAGE_FILTER:
		{
			UINT pid = *(UINT*) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->obd2_data_ctx.AddMessageFilter(vmsg,  pid ) )
			break;
		}
		case SUSI_API_CMD_OBD2_GET_MESSAGE_FILTER:
		{
			CHECK_RESULT( protocol_mgr->obd2_data_ctx.GetMessageFilter(vmsg) )
			break;
		}
		case SUSI_API_CMD_OBD2_REMOVE_MESSAGE_FILTER:
		{
			UINT pid = *(UINT*) cmd->input_parameter.front();

			CHECK_RESULT( protocol_mgr->obd2_data_ctx.RemoveMessageFilter(vmsg, pid ) )
			break;
		}
		case SUSI_API_CMD_OBD2_REMOVE_ALL_MESSAGE_FILTER:
		{
			CHECK_RESULT( protocol_mgr->obd2_data_ctx.RemoveAllMessageFilter(vmsg) )
			break;
		}
		default:
			break;
	}

	VCIL_LOG_D("SUSICommandHandler::cmd_handler_func_obd2 --  \r\n");
}

