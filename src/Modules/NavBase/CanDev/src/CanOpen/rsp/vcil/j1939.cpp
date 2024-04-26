#include <string.h>
#include "j1939.h"
#include "vmsg.h"
#include "DebugLog.h"

J1939Mgr::J1939Mgr()
{
}

J1939Mgr::~J1939Mgr()
{
}

USHORT J1939Mgr::Read( OUT PIMC_J1939_MSG_OBJECT object )
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;
	
	if( buffer.pop(object) < 0)
		return IMC_CAN_RX_NOT_READY;

	return IMC_ERR_NO_ERROR;
}

USHORT J1939Mgr::ReadTimedWait( OUT PIMC_J1939_MSG_OBJECT object , unsigned int ms)
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;
	
	if( buffer.pop_timed_wait(object, ms) < 0)
		return IMC_ERR_TIMEOUT;

	return IMC_ERR_NO_ERROR;
}

USHORT J1939Mgr::Write(VMSG *v, IN PIMC_J1939_MSG_OBJECT object )
{	
	if (object == NULL)
		return IMC_ERR_INVALID_POINTER;

	if ((object->pgn < 0) || (object->pgn > 0x1ffff))
		return IMC_ERR_INVALID_ARGUMENT;

	if (object->buf_len > MAX_J1939_MESSAGE_BUFFER_SIZE)
		return IMC_ERR_INVALID_ARGUMENT;

	if ((object->pri < 0) || (object->pri > 7))
		return IMC_ERR_INVALID_ARGUMENT;

	//VCIL_LOG_D("J1939Mgr::Write channel_number = 0x%x\r\n", object->channel_number);
	//VCIL_LOG_D("J1939Mgr::Write pgn = 0x%x\r\n", object->pgn);
	//VCIL_LOG_D("J1939Mgr::Write dst = 0x%x\r\n", object->dst);
	//VCIL_LOG_D("J1939Mgr::Write src = 0x%x\r\n", object->src);
	//VCIL_LOG_D("J1939Mgr::Write pri = 0x%x\r\n", object->pri);
	//VCIL_LOG_D("J1939Mgr::Write buf_len = %d\r\n", object->buf_len);

	unsigned char pf = (object-> pgn) >> 8;
	if(pf < 240)
	{
		object->pgn = pf;
		object->pgn <<= 8;
		object->pgn |= object->dst; 
	}

	int result = v->vmsg_tx_j1939 ( 0, object->pgn, object->dst, object->src, object->pri , object->buf_len, object->buf ) ;
	if ( result != SUCCESS )
	{
		return IMC_CAN_TX_WRITE_ERR;
	}
	
	return IMC_ERR_NO_ERROR;
}
USHORT J1939Mgr::SetEvent( void* hEvent )
{
	if (NULL == hEvent)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}	
	notify_event = (pthread_cond_t*) hEvent;

	return IMC_ERR_NO_ERROR;
}
USHORT J1939Mgr::AddMessageFilter(VMSG *v, UINT pgn )
{	
	if ((pgn < 0) || (pgn > 0x1ffff))
		return IMC_ERR_INVALID_ARGUMENT;
	
	int result = v->vmsg_add_filter_list_j1939( 0, pgn );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT J1939Mgr::GetMessageFilter(VMSG *v )
{	
	int result = v->vmsg_read_filter_list_j1939( 0 );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT J1939Mgr::RemoveMessageFilter(VMSG *v, UINT pgn )
{
	if ((pgn < 0) || (pgn > 0x1ffff))
		return IMC_ERR_INVALID_ARGUMENT;
	
	int result = v->vmsg_remove_filter_list_j1939( 0, pgn );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT J1939Mgr::RemoveAllMessageFilter (VMSG *v)
{
	int result = v->vmsg_remove_all_filter_j1939( 0);
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT J1939Mgr::SetTransmitConfig(VMSG *v, IMC_J1939_TRANSMIT_CONFIG transmit_config)
{
    if ( (transmit_config.source_address < 0) || 
		 (transmit_config.source_address > 255))
		return IMC_ERR_INVALID_ARGUMENT;

    int result = v->vmsg_set_address_and_name_j1939( 0, 
                                                  0x01, 
                                                  transmit_config.source_address, 
                                                  0x01, 
                                                  (unsigned char *) &transmit_config.source_name[0] );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;
    
    return IMC_ERR_NO_ERROR;
}

USHORT J1939Mgr::GetTransmitConfig(VMSG *v, PIMC_J1939_TRANSMIT_CONFIG ptransmit_config)
{
	if(ptransmit_config == NULL)
		return IMC_ERR_INVALID_POINTER;
		
    int result = v->vmsg_get_address_and_name_j1939( 0 );
    if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

    return IMC_ERR_NO_ERROR;
}
