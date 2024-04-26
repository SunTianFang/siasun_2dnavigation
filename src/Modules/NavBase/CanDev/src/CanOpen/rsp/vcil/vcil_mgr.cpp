#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "rsp/rsp_vcil/headers/SUSI_IMC.h"
#include "vcil_mgr.h"
#include "vmsg.h"
#include "DebugLog.h"
#include "vcil_port.h"

int VCILMgr::instance_number = 0;

VCILMgr::VCILMgr()
{
    cmd_mutex = PTHREAD_MUTEX_INITIALIZER;
    id = instance_number++;
};

VCILMgr::~VCILMgr()
{
	pthread_mutex_destroy(&cmd_mutex);
}

USHORT VCILMgr::init(const char* can_port, int baudrate)
{
	ULONG ret = 0;    
    VCIL_LOG_D("VCILMgr::init %s++ \r\n", can_port);
    
    port.Open(can_port, baudrate);
    
    vmsg = new VMSG(&port);

	data_ctx = new VehicleCommunicationMessage();
	cmd_handler = new SUSICommandHandler(data_ctx, vmsg);
	bus_message_handler = new BusMessageHandler(data_ctx, cmd_handler, &port);
	 
	//vmsg->vmsg_mode_receive(0x1F);
	//vmsg->vmsg_mode_receive(0x12);
	
    VCIL_LOG_D("VCILMgr::init -- \r\n");

	return IMC_ERR_NO_ERROR;
}

USHORT VCILMgr::deinit()
{
	if (bus_message_handler != NULL)
	{
		delete bus_message_handler;
	}
	
	if (cmd_handler != NULL)
	{
		delete cmd_handler;
	}

	if (data_ctx != NULL)
	{
		delete data_ctx;
	}
	
	if( vmsg != NULL)
		delete vmsg;
	
	port.Close();

	return IMC_ERR_NO_ERROR;
}

USHORT VCILMgr::ExecuteCmd(SUSICommand* cmd)
{
	//printf("%d)Attach cmd %u\n", id, cmd->susi_cmd);
	int ret = -1;
	pthread_mutex_lock(&cmd_mutex);
	cmd_handler->AttachCmdAndExcute(cmd);
	
	unsigned long ms = 3000; // maximum wait 1 sec
    struct timespec abs_time;
    clock_gettime(CLOCK_REALTIME, &abs_time);
    abs_time.tv_sec += 1;
    
    // wait command done
    /*
	pthread_mutex_lock(&cmd->ret_event_mutex);
    if( cmd->execute_done == false )
    {
    	ret = pthread_cond_timedwait(&cmd->ret_event, &cmd->ret_event_mutex, &abs_time);
    	if(ret != 0)
		{
			cmd->result = IMC_ERR_TIMEOUT;
		}
    }    
    pthread_mutex_unlock(&cmd->ret_event_mutex);
    */
    
    if(cmd->execute_done == false)
    {
    	ret = sem_timedwait(&cmd->ret_sem, &abs_time);
    	if(ret != 0)
		{
			cmd->result = IMC_ERR_TIMEOUT;
		}
    }
    
    cmd_handler->DetachCmd();
        
    pthread_mutex_unlock(&cmd_mutex);
}

