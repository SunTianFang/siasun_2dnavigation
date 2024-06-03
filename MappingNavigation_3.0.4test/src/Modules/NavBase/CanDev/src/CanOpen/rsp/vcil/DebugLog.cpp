#include "DebugLog.h"

#include <sys/select.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#define CONTROL_FILE_NAME "vcil_debug"
#define CONTROL_FILE_PATH "/tmp/" CONTROL_FILE_NAME
#define CONTROL_MAX_COMMAND_SIZE 512

#define CONTROL_FIFO_RECV_BUFF_SIZE 128

static pthread_t ThreadHandle;
static bool enable_thread = 0;

unsigned int debug_level = 0;

void load_debug_config()
{
	FILE *fp = fopen(CONTROL_FILE_NAME, "r");
	if( fp != NULL)
	{
		debug_level = VCIL_DEBUG_LOG_FLAG_DEFAULT;
		
		fclose(fp);
	}
}

void *background_control_monitor_process(void *p)
{
	int fd, ret;
	struct timeval time_limit;
	fd_set read_fds;
    
    if((mkfifo(CONTROL_FILE_PATH, 0666)<0)&&(errno!=EEXIST))
    {
    	return 0;
	}	
	
    fd = open(CONTROL_FILE_PATH, O_RDWR);

    char buff[CONTROL_FIFO_RECV_BUFF_SIZE];
    char control_cmd[CONTROL_MAX_COMMAND_SIZE+1] = {0};
    int control_len = 0;
    
    while(enable_thread)
    {
    	time_limit.tv_sec = 0;
		time_limit.tv_usec = 1000*100; // timeout 100 msec

		FD_ZERO(&read_fds);
		FD_SET(fd, &read_fds);

		ret = select(fd + 1, &read_fds, NULL, NULL, &time_limit);

		if(ret<0)
		{
			break;
		}
		else if(ret == 0)
		{
			// TIMEOUT
			continue;
		}
		else
		{
			if(FD_ISSET(fd, &read_fds))
			{
				ret = read( fd, buff, CONTROL_FIFO_RECV_BUFF_SIZE);
				if( ret == 0)
					continue;
			}
			else
			{
				break;
			}
		}
		
		for(int i=0;i<ret;i++)
		{
			if( control_len == CONTROL_MAX_COMMAND_SIZE)
				control_len = 0;
			
			if(buff[i] != '\n')
			{
				control_cmd[control_len++] = buff[i];
			}
			else
			{
				// handle command
				control_cmd[control_len++] = '\0';
				//printf("[CMD]>%s\n", control_cmd);
				
				if( sscanf(control_cmd, "debug=%u", &debug_level) == 1)
				{
					printf("debuglevel set 0x%08X\n", debug_level);
				}
				
				control_len =0;
			}
		}
    }
    
    close(fd);
    
    /* remove the FIFO */
    unlink(CONTROL_FILE_PATH);
}

void start_background_command_monitor()
{
	enable_thread = 1;
	pthread_attr_t ThreadAttr;
    pthread_attr_init(&ThreadAttr);
	pthread_create(&ThreadHandle, &ThreadAttr, &background_control_monitor_process, NULL);
	pthread_attr_destroy(&ThreadAttr);
}

void stop_background_command_monitor()
{
	if(enable_thread == 0)
		return;
	enable_thread = 0;
	pthread_join(ThreadHandle, NULL);
}
