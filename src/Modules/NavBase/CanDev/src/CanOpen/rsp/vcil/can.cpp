#include <stdio.h>
#include <string.h>
#include "common.h"
#include "vmsg.h"
#include "can.h"
#include "vcil_mgr.h"
#include "cmd_handler.h"
#include "DebugLog.h"

#include <unistd.h>

CanMgr::CanMgr()
{
}

CanMgr::~CanMgr()
{
}

USHORT CanMgr::SetBitTiming (VMSG *v, IN CAN_SPEED bit_rate, IN int silence_enable )
{
	int ret;
	unsigned char input_bit_rate = 0;

	switch ( bit_rate )
	{
		case CAN_SPEED_125K:
		{
			input_bit_rate = 0x05;
			break;
		}
		case CAN_SPEED_250K:
		{
			input_bit_rate = 0x03;
			break;
		}
		case CAN_SPEED_500K:
		{
			input_bit_rate = 0x02;
			break;
		}
		case CAN_SPEED_1M:
		{
			input_bit_rate = 0x00;
			break;
		}
		case CAN_SPEED_200K:
		{
			input_bit_rate = 0x04;
			break;
		}
		case CAN_SPEED_100K:
		{
			break;
		}
		default:
		{
			return IMC_ERR_INVALID_ARGUMENT;
			break;
		}
	}
	
	if( bit_rate <= CAN_SPEED_200K)
	{
		ret= v->vmsg_bitrate_can ( 0, input_bit_rate, silence_enable );
	}
	else
	{
		VCIL_LOG_V("vmsg_bitrate_can_btr command\n");
		if( bit_rate == CAN_SPEED_100K )
		{
			// SJW=1tq, BS1=12tq, BS2=7tq, Prescaler=18
			ret= v->vmsg_bitrate_can_btr ( 0, 0, 11, 7, 18, silence_enable);
		}
	}

	
	if( ret != SUCCESS)
	{
		return IMC_ERR_OPERATION_FAILED;
	}
	
	usleep(200);

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::Read(VMSG *v, PIMC_CAN_MSG_OBJECT object)
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;
		
    if( buffer.pop(object) < 0)
		return IMC_CAN_RX_NOT_READY;

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::ReadTimedWait(PIMC_CAN_MSG_OBJECT object, unsigned int ms)
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;

    if( buffer.pop_timed_wait(object, ms) < 0)
		return IMC_ERR_TIMEOUT;

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::Write(VMSG *v, IN PIMC_CAN_MSG_OBJECT object )
{
	unsigned char rtr;
	unsigned long id;

	if (object == NULL)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
    if ( (object->buf_len > 8) || ((object->message_type & 0x3) == 0x03) )
        return IMC_ERR_INVALID_ARGUMENT;    
    
	if (object->message_type & CAN_MESSAGE_STANDARD)
	{
		// CAN 2.0A maximum id is 0x7FF
		if (object->id > 0x7FF)
		{
		   	return IMC_ERR_INVALID_ARGUMENT;
		}
		
		id = object->id;
	}
	else if (object->message_type & CAN_MESSAGE_EXTENDED)
	{
		// CAN 2.0B maximum id is 0x1FFFFFFF
		if (object->id > 0x1FFFFFFF)
		{
		   	return IMC_ERR_INVALID_ARGUMENT;
		}
		
		id = object->id | 0x80000000;
	}
	else
	{
		// no defined any type of this message
		return IMC_ERR_INVALID_ARGUMENT;
	}

	int result;
	
	rtr = (object->message_type & CAN_MESSAGE_RTR) ? 0x01 : 0x00;
	rtr = rtr|0x80;	
	result = v->vmsg_tx_can_ex ( 0, rtr , id , object->buf_len, object->buf );	
	if ( result != SUCCESS )
	{
		VCIL_LOG_E("CanMgr::Write - ERROR  ! vmsg_tx_can fail !\r\n");
		return IMC_CAN_TX_WRITE_ERR;
	}
	
	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::SetEvent(void* hEvent )
{
	if (NULL == hEvent)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	notify_event = (pthread_cond_t*) hEvent;

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::SetMessageMask(VMSG *v, IN PIMC_CAN_MASK_OBJECT mask_object )
{
	unsigned char rtr = 0;
	unsigned char mode = 0;
	
	if (mask_object == NULL)
	{
		VCIL_LOG_E("CanMgr::SetMessageMask - ERROR ! Input object is null ! \r\n");
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	if ((mask_object->mask_number >= MAX_MASK_NUMBER) || ( mask_object->mask_number < 0) )//only had 0x00 ~ 0x0d
		return IMC_ERR_INVALID_ARGUMENT;

    if ((mask_object->message_type & CAN_MESSAGE_STANDARD) && ((mask_object->id < 0) || (mask_object->id > 0x7ff)))
    {
		return IMC_ERR_INVALID_ARGUMENT;
    }
	else if ((mask_object->message_type & CAN_MESSAGE_EXTENDED) && ((mask_object->id < 0) || (mask_object->id > 0x1fffffff)))
    {
		return IMC_ERR_INVALID_ARGUMENT;
    }

	if (mask_object->message_type & CAN_MESSAGE_RTR)
		rtr = 1;
	else
		rtr = 0;

	if (mask_object->message_type & CAN_MESSAGE_STANDARD)
		mode = CAN_MODE_A;
	else if (mask_object->message_type & CAN_MESSAGE_EXTENDED)
		mode = CAN_MODE_B;
	else
		return IMC_ERR_INVALID_ARGUMENT;

	if(SUCCESS != v->vmsg_add_filter_mask_can(0, 
										   mask_object->mask_number, 
										   rtr,
										   mode,
										   mask_object->id,
										   mask_object->mask))
	{
		VCIL_LOG_E("CanMgr::SetMessageMask - ERROR ! vmsg_add_filter_mask_can fail ! \r\n");
		return IMC_ERR_OPERATION_FAILED;
	}
	
	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::GetMessageMask(VMSG *v, OUT PIMC_CAN_MASK_OBJECT mask_object)
{
	if (mask_object == NULL)
	{
		VCIL_LOG_E("CanMgr::GetMessageMask - ERROR ! Input object is null ! \r\n");
		return IMC_ERR_INVALID_ARGUMENT;
	}

	if ((mask_object->mask_number < 0 ) || (mask_object->mask_number >=MAX_MASK_NUMBER))
		return IMC_ERR_INVALID_ARGUMENT;

    if( SUCCESS != v->vmsg_read_filter_mask_can( 0, mask_object->mask_number ) )
    {
		VCIL_LOG_E("CanMgr::GetMessageMask - ERROR ! vmsg_read_filter_mask_can fail ! \r\n");
		return IMC_ERR_OPERATION_FAILED;
	}

    return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::RemoveMessageMask (VMSG *v, IN int mask_number )
{	
	if ((mask_number < 0 ) || (mask_number >=MAX_MASK_NUMBER))
		return IMC_ERR_INVALID_ARGUMENT;
	
	if(SUCCESS != v->vmsg_remove_filter_mask_can(0, mask_number))
	{
		VCIL_LOG_E("CanMgr::RemoveMessageMask - ERROR ! vmsg_remove_filter_mask_can fail ! \r\n");
		return IMC_ERR_OPERATION_FAILED;
	}

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::ResetMessageMask(VMSG *v )
{
    if (SUCCESS != v->vmsg_clear_filter_mask_can(0))
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::GetErrorStatus(VMSG *v )
{
    if (SUCCESS != v->vmsg_read_can_esr(0))
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT CanMgr::GetBitTiming(VMSG *v)
{

    if (SUCCESS != v->vmsg_read_can_bitrate(0))
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
///////////////////////////////////////////////////////////////////////////////
