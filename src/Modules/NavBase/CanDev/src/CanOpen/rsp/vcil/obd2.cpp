#include <string.h>
#include "obd2.h"
#include "vmsg.h"
#include "DebugLog.h"

OBD2Mgr::OBD2Mgr()
{
}

OBD2Mgr::~OBD2Mgr()
{
}

USHORT OBD2Mgr::Read( OUT PIMC_OBD2_MSG_OBJECT object )
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;
	
	if( buffer.pop(object) < 0)
		return IMC_CAN_RX_NOT_READY;

	return IMC_ERR_NO_ERROR;
}

USHORT OBD2Mgr::ReadTimedWait( OUT PIMC_OBD2_MSG_OBJECT object , unsigned int ms)
{
	if (object == NULL)
		return IMC_ERR_INVALID_ARGUMENT;
	
	if( buffer.pop_timed_wait(object, ms) < 0)
		return IMC_ERR_TIMEOUT;

	return IMC_ERR_NO_ERROR;
}

USHORT OBD2Mgr::Write(VMSG *v, IN PIMC_OBD2_MSG_OBJECT object )
{
	if (object == NULL)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}

    if ((object->pri < 0) || (object->pri > 7))
		return IMC_ERR_INVALID_ARGUMENT;

    if ((object->tat != 218) && (object->tat != 219))
		return IMC_ERR_INVALID_ARGUMENT;

    if ((object->buf_len < 0) || (object->buf_len > MAX_OBD2_MESSAGE_BUFFER_SIZE))
		return IMC_ERR_INVALID_ARGUMENT;

    int result = v->vmsg_tx_obd2 ( 0, object->dst, object->src, object->pri , object->tat, (unsigned char) object->buf_len, object->buf ) ;
	if ( result != SUCCESS )
	{
		return IMC_CAN_TX_WRITE_ERR;
	}
	
	return IMC_ERR_NO_ERROR;
}
USHORT OBD2Mgr::SetEvent( void* hEvent )
{
	if (NULL == hEvent)
	{
		return IMC_ERR_INVALID_ARGUMENT;
	}
	
	notify_event = (pthread_cond_t*) hEvent;
	return IMC_ERR_NO_ERROR;
}
USHORT OBD2Mgr::AddMessageFilter(VMSG *v, UINT pid )
{
	if ((pid < 0) || (pid > 0xff))
		return IMC_ERR_INVALID_ARGUMENT;
	
	int result = v->vmsg_add_filter_list_obd2( 0, pid );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT OBD2Mgr::GetMessageFilter(VMSG *v )
{
	int result = v->vmsg_read_filter_list_obd2( 0);
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT OBD2Mgr::RemoveMessageFilter(VMSG *v, UINT pid )
{
	if ((pid < 0) || (pid > 0xff))
		return IMC_ERR_INVALID_ARGUMENT;
	
	int result = v->vmsg_remove_filter_list_obd2( 0, pid );
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
USHORT OBD2Mgr::RemoveAllMessageFilter (VMSG *v )
{
	int result = v->vmsg_remove_all_filter_obd2( 0);
	if (SUCCESS != result)
		return IMC_ERR_OPERATION_FAILED;

	return IMC_ERR_NO_ERROR;
}
