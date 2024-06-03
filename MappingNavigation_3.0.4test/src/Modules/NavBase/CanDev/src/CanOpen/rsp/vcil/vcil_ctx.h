#ifndef _VCIL_CTX_H
#define _VCIL_CTX_H

#include <pthread.h>
#include "circular_buffer.h"
#include "vcil_vmsg.h"

class VehicleCommunicationMessageContent
{
public:
	VehicleCommunicationMessageContent();	
	virtual ~VehicleCommunicationMessageContent();
	
	unsigned long total_recv_valid_msgs;
	unsigned long total_recv_invalid_bytes;
	
	pthread_cond_t *notify_event;
	pthread_mutex_t *notify_event_mutux;
};

#endif
