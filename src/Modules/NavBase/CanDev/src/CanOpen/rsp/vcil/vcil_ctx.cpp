#include "vcil_ctx.h"

VehicleCommunicationMessageContent::VehicleCommunicationMessageContent()
:total_recv_valid_msgs(0)
,total_recv_invalid_bytes(0)
,notify_event(NULL)
,notify_event_mutux(NULL)
{
}

VehicleCommunicationMessageContent::~VehicleCommunicationMessageContent()
{
}

