#include <string.h>
#include "bus_msg_handler.h"

#include "can.h"
#include "vmsg.h"
#include "DebugLog.h"

#include <sys/time.h>

#define BUF_RX_SIZE   1024

#define MESSAGE_HANDLER_PRIORITY    (50)

BusMessageHandler::BusMessageHandler(VehicleCommunicationMessage *vcm, SUSICommandHandler *c, VCILPort *p)
{
	protocol_mgr = vcm;
	cmder = c;
	port = p;
	
	pthread_attr_t ThreadAttr;
    pthread_attr_init(&ThreadAttr);
		
    //QianYizhou Add
    struct sched_param sched;
    sched.sched_priority = MESSAGE_HANDLER_PRIORITY;
    VCIL_LOG_D( "[CAN SDK]set the BusMessageHandler's priority: %d\n", sched.sched_priority );
    pthread_attr_setinheritsched( &ThreadAttr, PTHREAD_EXPLICIT_SCHED );
    pthread_attr_setschedpolicy( &ThreadAttr, SCHED_FIFO );
    pthread_attr_setschedparam( &ThreadAttr, &sched );
    //Add Ended

	pthread_create(&ThreadHandle, &ThreadAttr, &BusMessageHandler::BusMessageHandleThread, this);
	pthread_attr_destroy(&ThreadAttr);
}

BusMessageHandler::~BusMessageHandler()
{
	StopThread();
}

void BusMessageHandler::StopThread()
{
	if( !enable )
		return;
	enable = false;
	VCIL_LOG_D("Wait thread\n");
	pthread_cancel(ThreadHandle);
	//pthread_join(ThreadHandle, NULL);
	VCIL_LOG_D("Wait thread-done\n");
}

void* BusMessageHandler::BusMessageHandleThread( void* lpParam )
{
	DWORD ret = 0;
	unsigned char payload[BUF_RX_SIZE];
	unsigned char bFWVersion[2] = {0x0, 0x0};

	IMC_CAN_MSG_OBJECT can_obj = {'\0'};
	IMC_J1939_MSG_OBJECT j1939_obj = {'\0'};
	IMC_OBD2_MSG_OBJECT obd2_obj = {'\0'};

	BusMessageHandler *This = (BusMessageHandler *)lpParam;
	VehicleCommunicationMessage *vmsg_mgr = This->protocol_mgr;
	
	This->enable = true;
	
	VCIL_LOG_D("Recv thread start..\n");

	while(1)
	{
		RPACKET packet;

		if (This->enable == false)
		{
			VCIL_LOG_V("Not enable\n");
			break;
		}
					
		//init_RRPACKET ( &packet, payload, sizeof ( payload ) );
		packet.payload = payload;
		packet.payload_length = BUF_RX_SIZE;
		ret = This->port->Recv_packet ( &packet, TIMEOUT_MS );
		if ( ret != SUCCESS )
		{
			if( ret != ERR_TIMEOUT)
				VCIL_LOG_V("Recv package fail = %d\n", (int)ret);
			continue;
		}

		if(bFWVersion[0] == 0 && packet.cmd!=CMD_VMSG_FIRMWARE_VERSION_RESP)
		{
			VCIL_LOG_V("wait firmware version\n");
			continue;
		}
								
		switch (packet.cmd)
		{
			case CMD_VMSG_RX_CAN:
			{
				memset( &can_obj , '\0' ,sizeof(can_obj));
				
				can_obj.id = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];
				can_obj.buf_len =  packet.length - 7;
				for (int idx=0; idx < can_obj.buf_len; idx++)
					can_obj.buf[idx] = payload[5 + idx];
				

				if (! (can_obj.id & 0x80000000))
				{
					can_obj.message_type = CAN_MESSAGE_STANDARD;
				}
				else
				{
					can_obj.message_type = CAN_MESSAGE_EXTENDED;
					can_obj.id = can_obj.id & (~0x80000000);
				}
				
				vmsg_mgr->can_data_ctx.buffer.push(can_obj);

				if (vmsg_mgr->can_data_ctx.notify_event != NULL)
				{
					pthread_cond_signal(vmsg_mgr->can_data_ctx.notify_event);
				}

				break;
			}
			case CMD_VMSG_RX_CAN_EX:
			{
				if(payload[1]&0x80)
				{
					This->cmder->CmdDoneNotify(IMC_ERR_NO_ERROR);
				}
				else
				{
					memset( &can_obj , '\0' ,sizeof(can_obj));
					can_obj.id = (payload[2] << 24) | (payload[3] << 16) | (payload[4] << 8) | payload[5];
						can_obj.buf_len =  packet.length - 8;
						for (int idx=0; idx < can_obj.buf_len; idx++)
							can_obj.buf[idx] = payload[6 + idx];

					if (! (can_obj.id & 0x80000000))
					{
						can_obj.message_type = CAN_MESSAGE_STANDARD;
					}
					else
					{
						can_obj.message_type = CAN_MESSAGE_EXTENDED;
						can_obj.id = can_obj.id & (~0x80000000);
					}

					if(payload[1]&0x02)
						can_obj.message_type =  (CAN_MESSAGE_TYPE)( (int)can_obj.message_type | (int)CAN_MESSAGE_RTR);				
				
					vmsg_mgr->can_data_ctx.buffer.push(can_obj);

					if (vmsg_mgr->can_data_ctx.notify_event != NULL)
					{
						pthread_cond_signal(vmsg_mgr->can_data_ctx.notify_event);
					}
				}
				break;
			}
			case CMD_VMSG_RX_J1939:
			{
				memset( &j1939_obj , '\0' ,sizeof(j1939_obj));

				j1939_obj.pgn = (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
				j1939_obj.dst = payload[4];
				j1939_obj.src = payload[5];
				j1939_obj.pri = payload[6];
				j1939_obj.buf_len = packet.length - 9;

				for (int idx = 0; idx < packet.length - 9; idx++)
				{
					j1939_obj.buf[idx] = payload[idx + 7];
				//	VCIL_LOG_D("idx = %d, %02lx\r\n",idx,j1939_obj.buf[idx]);
				}
				
				vmsg_mgr->j1939_data_ctx.buffer.push(j1939_obj);

				if (vmsg_mgr->j1939_data_ctx.notify_event != NULL)
				{
                    pthread_cond_signal( vmsg_mgr->j1939_data_ctx.notify_event );
				}

				break;
			}
            case CMD_VMSG_RX_OBD2:
            {
				memset( &obd2_obj , '\0' ,sizeof(obd2_obj));

				obd2_obj.dst = payload[1];
				obd2_obj.src = payload[2];
				obd2_obj.pri = payload[3];
                obd2_obj.tat = payload[4];
				obd2_obj.buf_len = packet.length - 7;

				for (int idx = 0; idx < packet.length - 7; idx++)
				{
					obd2_obj.buf[idx] = payload[idx + 5];
					//VCIL_LOG_D("idx = %d, %02x\r\n",idx,obd2_obj.buf[idx]);
				}
				
				vmsg_mgr->obd2_data_ctx.buffer.push(obd2_obj);

				if (vmsg_mgr->obd2_data_ctx.notify_event != NULL)
				{
                    pthread_cond_signal( vmsg_mgr->obd2_data_ctx.notify_event );
				}
                break;
            }
			case CMD_VMSG_FIRMWARE_VERSION_RESP:
			{
				VCIL_LOG_D("CMD_VMSG_FIRMWARE_VERSION_RESP++\n");
				if( bFWVersion[0] == 0)
				{
					VCIL_LOG_D("CMD_VMSG_FIRMWARE_VERSION_RESP++ first init\n");
					// first init
					bFWVersion[0] = payload[0];
					bFWVersion[1] = payload[1];

					// set VMSG_TX_CAN Type
					if(bFWVersion[0] >1)
						vmsg_mgr->can_data_ctx.tx_type = 1;
				}
				else
				{
					VCIL_LOG_D("CMD_VMSG_FIRMWARE_VERSION_RESP++ second\n");
					bFWVersion[0] = payload[0];
					bFWVersion[1] = payload[1];
					char *version = (char *) This->cmder->NowRunCmd->input_parameter.front();
					
					sprintf(version, "%d.%d", bFWVersion[0], bFWVersion[1]);
				}

				This->cmder->CmdDoneNotify(IMC_ERR_NO_ERROR);

				VCIL_LOG_D("CMD_VMSG_FIRMWARE_VERSION_RESP--\n");
				break;
			}
            case CMD_VMSG_READ_FILTER_MASK_CAN_RESP:
            {
                USHORT ret = IMC_ERR_INVALID_POINTER;
                PIMC_CAN_MASK_OBJECT mask_object = (PIMC_CAN_MASK_OBJECT) This->cmder->NowRunCmd->input_parameter.front();

                if (payload[2] == 0x00)
                    mask_object->message_type = CAN_MESSAGE_STANDARD;
                else
                    mask_object->message_type = CAN_MESSAGE_EXTENDED;

                if (payload[3])
                    mask_object->message_type = (CAN_MESSAGE_TYPE) (mask_object->message_type | CAN_MESSAGE_RTR);
                else
                    mask_object->message_type = (CAN_MESSAGE_TYPE) (mask_object->message_type & (~CAN_MESSAGE_RTR));

                if (mask_object->message_type & CAN_MESSAGE_STANDARD)
                {
                    mask_object->id = (payload[4] << 8) | (payload[5]);
                    mask_object->mask = (payload[8] << 8) | (payload[9]);
                }
                else
                {
                    mask_object->id = (payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | payload[7];
                    mask_object->mask = (payload[8] << 24) | (payload[9] << 16) | (payload[10] << 8) | payload[11];
                }

                This->cmder->CmdDoneNotify(IMC_ERR_NO_ERROR);
                break;
            }
			case CMD_VMSG_READ_FILTER_LIST_J1939_RESP:
			{
				/*
					This function had 2 function.
					1. Get total : If mid was null, we only return the total.
					2. Get mid : If mid was not null and input total equal recv total, we will copy mid to output buffer.
				*/
				USHORT ret = IMC_ERR_INVALID_POINTER;
				UINT *total = (UINT*) This->cmder->NowRunCmd->input_parameter.front(); This->cmder->NowRunCmd->input_parameter.pop_front();
				UINT *pgn = (UINT*) This->cmder->NowRunCmd->input_parameter.front();
				UINT total_recv = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];

				if ((total != NULL) && (pgn == NULL))
				{
					*total = total_recv;
					VCIL_LOG_D("BusMessageHandler::BusMessageHandleThread - J1939 filter total = %d \r\n", *total);
					ret = IMC_ERR_NO_ERROR;
				}

				if ((total != NULL) && (pgn != NULL) && ( *total == total_recv))
				{
					try
					{
						for(UINT i = 0; i< *total; i++)
						{
							pgn[i] = ((payload[5 + 3*i] << 16) | (payload[5 + 3*i + 1] << 8) | (payload[5 + 3*i + 2]));
						}
					}
					catch(const char* message)
					{
						VCIL_LOG_E("BusMessageHandler::BusMessageHandleThread - ERROR ! J1939 get filter fail = %s \r\n", message);
						This->cmder->NowRunCmd->result = IMC_ERR_INVALID_POINTER;
						break;
					}
					ret = IMC_ERR_NO_ERROR;
				}

				This->cmder->CmdDoneNotify(ret);
				break;
			}
			case CMD_VMSG_READ_CAN_ESR_RESP:
			{
				USHORT ret = IMC_ERR_INVALID_POINTER;
				PIMC_CAN_ERROR_STATUS_OBJECT obj = (PIMC_CAN_ERROR_STATUS_OBJECT) This->cmder->NowRunCmd->input_parameter.front();

				UINT rec = payload[1];
				UINT tec = payload[2];
				UINT lec = payload[3];
				UINT flag = payload[4];

				if( obj != NULL)
				{
					obj->rec = rec;
					obj->tec = tec;
					obj->last_error_code = lec;
					obj->error_flag = flag;
					ret = IMC_ERR_NO_ERROR;
				}

				This->cmder->CmdDoneNotify(ret);
				break;
			}

			case CMD_VMSG_READ_BITRATE_CAN_RESP:
			{
				USHORT ret = IMC_ERR_INVALID_POINTER;
				CAN_SPEED *bitrate = (CAN_SPEED *) This->cmder->NowRunCmd->input_parameter.front(); This->cmder->NowRunCmd->input_parameter.pop_front();
				CAN_BUS_MODE *mode = (CAN_BUS_MODE *) This->cmder->NowRunCmd->input_parameter.front(); This->cmder->NowRunCmd->input_parameter.pop_front();

				if( (bitrate != NULL) && (mode != NULL) )
				{
					switch( (CAN_SPEED)payload[1])
					{
						case 0:
							*bitrate = CAN_SPEED_1M;
							break;

						case 2:
							*bitrate = CAN_SPEED_500K;
							break;

						case 3:
							*bitrate = CAN_SPEED_250K;
							break;

						case 4:
							*bitrate = CAN_SPEED_200K;
							break;

						case 5:
							*bitrate = CAN_SPEED_125K;
							break;

						case 0xFE:
							*bitrate = CAN_SPEED_INIT;
							break;

						default:
							*bitrate = CAN_SPEED_USER_DEFINE;
							break;

					}

					*mode = (CAN_BUS_MODE)payload[2];
					ret = IMC_ERR_NO_ERROR;
				}

				This->cmder->CmdDoneNotify(ret);
				break;
			}
            case CMD_VMSG_READ_FILTER_LIST_OBD2_RESP:
            {
				/*
					This function had 2 function.
					1. Get total : If pid was null, we only return the total.
					2. Get mid : If pid was not null and input total equal recv total, we will copy pid to output buffer.
				*/
				USHORT ret = IMC_ERR_INVALID_POINTER;
				UINT *total = (UINT*) This->cmder->NowRunCmd->input_parameter.front(); This->cmder->NowRunCmd->input_parameter.pop_front();
				UINT *pid = (UINT*) This->cmder->NowRunCmd->input_parameter.front();
				UINT total_recv = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];

				if ((total != NULL) && (pid == NULL))
				{
					*total = total_recv;
					VCIL_LOG_D("BusMessageHandler::BusMessageHandleThread - OBD2 filter total = %d \r\n", *total);
					ret = IMC_ERR_NO_ERROR;
				}

				if ((total != NULL) && (pid != NULL) && ( *total == total_recv))
				{
					try
					{
						for(UINT i = 0; i< *total; i++)
						{
							pid[i] = payload[5 + i];
						}
					}
					catch(const char* message)
					{
						VCIL_LOG_E("BusMessageHandler::BusMessageHandleThread - ERROR ! OBD2 get filter fail = %s \r\n", message);
						This->cmder->NowRunCmd->result = IMC_ERR_INVALID_POINTER;
						break;
					}
					ret = IMC_ERR_NO_ERROR;
				}

				This->cmder->CmdDoneNotify(ret);
				break;
            }
            case CMD_VMSG_GET_ADDRESS_AND_NAME_J1939_RESP:
            {
                USHORT ret = IMC_ERR_INVALID_POINTER;
                PIMC_J1939_TRANSMIT_CONFIG ptransmit_config = (PIMC_J1939_TRANSMIT_CONFIG) This->cmder->NowRunCmd->input_parameter.front();

                ptransmit_config->source_address = payload[1];

                for(int i = 0; i < 8 ;i++)
                {
                    ptransmit_config->source_name[i] = payload[i + 2];

                }

                This->cmder->CmdDoneNotify(IMC_ERR_NO_ERROR);
                break;
            }
			case ACK_VMSG_CAN:
			{
				if (payload[0] == 0 )
				{
					This->cmder->CmdDoneNotify(IMC_ERR_NO_ERROR);
				}
				else if(payload[0] == 2)
				{
					This->cmder->CmdDoneNotify(IMC_CAN_COMMAND_TIMEOUT_ERROR);
				}
				else
				{
					VCIL_LOG_V("ACK fail = %d\n", payload[0]);
					This->cmder->CmdDoneNotify(IMC_ERR_OPERATION_FAILED);
				}
				break;
			}

			default:
			{
				VCIL_LOG_E("BusMessageHandleThread - unknown VMSG = 0x%02X\r\n", packet.cmd);
				break;
			}
		}

	}

	VCIL_LOG_D("Recv thread stop..\n");

	return 0;
}
