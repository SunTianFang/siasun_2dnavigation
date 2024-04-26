#include <string.h>
#include "j1708.h"
#include "vmsg.h"
#include "DebugLog.h"

J1708Mgr::J1708Mgr()
{
}

J1708Mgr::~J1708Mgr()
{
}

USHORT J1708Mgr::Read( OUT PIMC_J1708_MSG_OBJECT object )
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;

    if( buffer.pop(object) < 0)
		return IMC_CAN_RX_NOT_READY;

	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::Write( IN PIMC_J1708_MSG_OBJECT object )
{
	if (object == NULL)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
    
    if ((object->pri < 1) || (object->pri > 8))
    {
        return IMC_ERR_INVALID_ARGUMENT;
    }
    
    // 255 , 511 is specail page extend pid not use 
    if( (object->mid > 0xFF) || (object->pid > 510) || (object->pid == 0xFF))
    {
    	return IMC_ERR_INVALID_ARGUMENT;
    }
    
    // J1708 the maximum length should not exceed 21 over MID + DATA + CHECKSUM
    // 21 - MID(1) - CHECKSUM(1) - PID(1~2) 
    if( (object->pid > 0xFF) && (object->buf_len > 17))
    {
    	return IMC_ERR_INVALID_ARGUMENT;
    } else if( (object->pid < 0xFF) && (object->buf_len > 18))
    {
    	return IMC_ERR_INVALID_ARGUMENT;
    }
    
	int result = vmsg_tx_j1708 ( object->mid, object->pid, object->pri , object->buf_len, object->buf ) ;
	if ( result != SUCCESS )
	{
		return IMC_CAN_TX_WRITE_ERR;
	}
	
	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::SetEvent( void* hEvent )
{
	if (NULL == hEvent)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	notify_event = (pthread_cond_t*) hEvent;

	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::AddMessageFilter( IN UINT mid )
{
	if( mid > 0xFF)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	int result = vmsg_add_filter_list_j1708( mid );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::GetMessageFilter( )
{
	int result = vmsg_read_filter_list_j1708();
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::RemoveMessageFilter( IN UINT mid )
{
	if( mid > 0xFF)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	int result = vmsg_remove_filter_list_j1708( mid );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}

USHORT J1708Mgr::RemoveAllMessageFilter ( void )
{
	int result = vmsg_remove_all_filter_j1708();
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;	
	return IMC_ERR_NO_ERROR;
}
