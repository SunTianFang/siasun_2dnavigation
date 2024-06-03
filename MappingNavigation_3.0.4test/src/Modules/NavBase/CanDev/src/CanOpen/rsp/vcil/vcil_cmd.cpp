#include "vcil_cmd.h"
#include "DebugLog.h"

SUSI_CMD_GROUP g_susi_api_cmd_table[] = {
	CMD_GROUP_VCIL,
	CMD_GROUP_VCIL,
	CMD_GROUP_VCIL,
	CMD_GROUP_VCIL,
	CMD_GROUP_VCIL,
	CMD_GROUP_VCIL,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_CAN,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1708,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1939,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_J1587,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2,
	CMD_GROUP_OBD2	
};

SUSICommand::SUSICommand(SUSI_API_CMD cmd)
:execute_done(false)
//,ret_event(PTHREAD_COND_INITIALIZER)
//,ret_event_mutex(PTHREAD_MUTEX_INITIALIZER)
{
	cmd_group = g_susi_api_cmd_table[cmd];
	susi_cmd = cmd;
	
	sem_init(&ret_sem, 0, 0);
}

SUSICommand::~SUSICommand()
{
    //pthread_cond_destroy(&ret_event);
    //pthread_mutex_destroy (&ret_event_mutex);	
    sem_destroy(&ret_sem);
}

void SUSICommand::Done(USHORT ret_val)
{
	//pthread_mutex_lock(&ret_event_mutex);
	result = ret_val;
    execute_done = true;
    //pthread_cond_signal(&ret_event);
    //pthread_mutex_unlock(&ret_event_mutex);
    sem_post(&ret_sem);
}

