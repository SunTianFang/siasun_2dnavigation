#include "vcil_port.h"

#include "unistd.h"
#include <sys/select.h> // for select
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <termios.h>

#include <sys/time.h>

#include "DebugLog.h"
#include "vmsg.h"
#include <string.h>

#define STM32_DEFAULT_UART_BAUDRATE			460800
#define STM32_UART_UART_BAUDRATE			921600

//#define DEBUG_RAW_DATA

VCILPort::VCILPort()
:vcil_device_port(-1)
,baudrate(STM32_DEFAULT_UART_BAUDRATE)
,is_open(false)
,recv_size(0)
,recv_index(0)
,port_id(-1)
{
	wbuf_mutex = PTHREAD_MUTEX_INITIALIZER;
}

VCILPort::~VCILPort()
{
	pthread_mutex_destroy(&wbuf_mutex);
}

USHORT VCILPort::Open(const char *path, int init_baudrate)
{
	sprintf(node_path, "%s", path);
	baudrate = init_baudrate;
	sscanf(path, "/dev/ttyS%d", &port_id);
	Open();
}

BOOL VCILPort::SetBaudRate(int hCOMPort, DWORD dwBaudRate)
{
	struct termios cfg;
	int baud_rate;

	if (tcgetattr(hCOMPort, &cfg))
	{
		VCIL_LOG_D("[SP_Debug] tcgetattr() failed \n");
		close(hCOMPort);
		return false;
	}
		
	switch(dwBaudRate)
	{
		case 300:
			baud_rate = B300;
		break;
		case 600:
			baud_rate = B600;
		break;
		case 1800:
			baud_rate = B1800;
		break;
		case 2400:
			baud_rate = B2400;
		break;
		case 4800:
			baud_rate = B4800;
		break;
		case 9600:
			baud_rate = B9600;
		break;
		case 19200:
			baud_rate = B19200;
		break;
		case 38400:
			baud_rate = B38400;
		break;
		case 57600:
			baud_rate = B57600;
		break;
		case 115200:
			baud_rate = B115200;
		break;
		case 230400:
			baud_rate = B230400;
		break;
		case 460800:
			baud_rate = B460800;
		break;
		case 921600:
			baud_rate = B921600;
		break;
		case 1500000:
			baud_rate = B1500000;
		break;
		default :
			VCIL_LOG_D("error dwBaudRate = %d \n",(int)dwBaudRate);
		return FALSE;
	}
		
	cfmakeraw(&cfg);
	cfsetispeed(&cfg, baud_rate);
	cfsetospeed(&cfg, baud_rate);
	
	//cfg.c_cc[VMIN] = 1;
	//cfg.c_cc[VTIME] = 0;

	if (tcsetattr(hCOMPort, TCSANOW, &cfg))
	{
		VCIL_LOG_D("tcsetattr() failed\n");
		return false;
	}
	return true;
}

USHORT VCILPort::Open()
{
	if( is_open)
		Close();
	
	VCIL_LOG_D("Open Baudrate %d\n", baudrate);
	 
    vcil_device_port = open(node_path, O_RDWR);
    VCIL_LOG_D("Open device number %d %s\n", vcil_device_port, node_path);
	if ( vcil_device_port < 0 )
	{
		VCIL_LOG_E("CarBusMgr::init - ERROR Fails to open 460800 COM port %s\r\n", node_path);
		return IMC_ERR_DRIVER_OPEN;
	}
		
    if(SetBaudRate(vcil_device_port, STM32_DEFAULT_UART_BAUDRATE) != true)
	{
        VCIL_LOG_E("CarBusMgr::init - ERROR Fails to set 460800 COM port %s\r\n", node_path);
		close( vcil_device_port );
		return IMC_ERR_DRIVER_OPEN;
	}

	// sleep 300ms avoid switch com port not ready
	usleep(300000);
	
	if(baudrate == STM32_UART_UART_BAUDRATE)
	{
		RPACKET cfg_packet;
		unsigned char wpayload[2];

		init_WRPACKET ( &cfg_packet, CMD_VMSG_BAUDRATE_MODULE, wpayload, sizeof ( wpayload ) );
		wpayload[0] = BAUDRATE_MODULE_921600;		
		wpayload[1] = 0x00;

		Send_packet_no_wait ( &cfg_packet );
			
		close( vcil_device_port );
	
		// sleep 300ms avoid switch com port not ready
		usleep(300000);
	
		vcil_device_port = open(node_path, O_RDWR);
		if ( vcil_device_port < 0 )
		{
			VCIL_LOG_E("CarBusMgr::init - ERROR Fails to open 921600 COM port %s\r\n", node_path);
			return IMC_ERR_DRIVER_OPEN;
		}
	
		if(SetBaudRate(vcil_device_port, STM32_UART_UART_BAUDRATE) != true)
		{
		    VCIL_LOG_E("CarBusMgr::init - ERROR Fails to set 921600 COM port %s\r\n", node_path);
			close( vcil_device_port );
			return IMC_ERR_DRIVER_OPEN;
		}
	}
	
	wbuf_front = 0;
	wbuf_rear = 0;
	wbuf_size = 0;
	
	is_open = true;
#ifdef DEBUG_RAW_DATA
	debug_raw_file = fopen(strstr(node_path, "tty"), "w");
	if( debug_raw_file == NULL)
	{
		printf("open file error\n");
	}
#endif
}

void VCILPort::Close()
{
    int ret = close( vcil_device_port );
    if(ret !=0)
    {
    	VCIL_LOG_E("close fail = %d\n", errno);
    }
#ifdef DEBUG_RAW_DATA    
    if( debug_raw_file )
    	fclose(debug_raw_file);
#endif
    
    is_open = false;
}

void VCILPort::Purge()
{
    VCIL_LOG_D("vcil_fun_clean_buffer++ \n");
    while(1)
    {
        int ret;
        char clean_buf[512] = {'\0'};
        struct timeval time_limit;    
        fd_set read_fds, write_fds, except_fds;
    
        FD_ZERO(&read_fds);
        FD_ZERO(&write_fds);
        FD_ZERO(&except_fds);
        FD_SET(vcil_device_port, &read_fds);
            
        time_limit.tv_sec = 0;
        time_limit.tv_usec = 1000 * 10;

        if (select(vcil_device_port + 1, &read_fds, &write_fds, &except_fds, &time_limit) == 1)
        {
            read( vcil_device_port, clean_buf, sizeof(clean_buf));
        }
        else
        {
           break;
        }
    }
    VCIL_LOG_D("vcil_fun_clean_buffer-- \n");
}

int VCILPort::RecvByte ( unsigned char &b,unsigned int timeout)
{
	//int write_time = 0;
	
	//static BYTE recv_buffer[RECV_BUFFER_SIZE];
	//static int recv_size = 0;
	//static int recv_index = 0;

	int ret;
	struct timeval time_limit;
	fd_set read_fds;

	unsigned long ulRtn;
	unsigned long BytesRead;	

	if(recv_index==recv_size)
	{
		/*
		time_limit.tv_sec = 0;
		time_limit.tv_usec = 1000 * timeout;

		FD_ZERO(&read_fds);
		FD_SET(vcil_device_port, &read_fds);

		ret = select(vcil_device_port + 1, &read_fds, NULL, NULL, &time_limit);

		if(ret<0)
		{
			return ERR_READ_PORT;
		}
		else if(ret == 0)
		{
			// TIMEOUT
			return ERR_EMPTY_DATA;
		}
		else
		{
			if(FD_ISSET(vcil_device_port, &read_fds))
			{
				ulRtn = read( vcil_device_port, recv_buffer, RECV_BUFFER_SIZE);
			}
			else
			{
				return ERR_READ_PORT;
			}
		}
		*/
		ulRtn = read( vcil_device_port, recv_buffer, RECV_BUFFER_SIZE);
		
		if ( ulRtn < 0 )
			return ERR_READ_PORT;
		else if ( ulRtn == 0 )
		{
			return ERR_EMPTY_DATA;
		}

		recv_size = ulRtn;
		recv_index = 0;
	}

	if(recv_size==recv_index)
		return ERR_READ_PORT;

	b = recv_buffer[recv_index++];
#ifdef DEBUG_RX	
	if( b != 0xC0 )
		printf("%02x,", b);
	else
		printf("\n%02x,", b);
#endif

	return SUCCESS;
}

int VCILPort::Send ( unsigned char *buf, unsigned long len )
{	
    int ret;
    int needbytewrite = len;
    unsigned char *buf_head = buf;
 
    while(needbytewrite!=0)
    {
		ret = write( vcil_device_port, buf_head, needbytewrite);
		if (ret <= 0)
		{
			VCIL_LOG_V("Error %d device %d\n", errno, vcil_device_port);
		    return ERR_WRITE_PORT;
		}
		needbytewrite-=ret;
		buf_head+=ret;		
    }

#if 1
	if( VCIL_DEBUG_LOG_FLAG_VMSG & debug_level )
	{
	   	VCIL_LOG_V ( "[%d]TX[%lu]: " , port_id, len);
		for ( unsigned int idx = 0 ; idx < len ; ++idx )
		{
			VCIL_LOG_V ( "0x%02X ", buf[idx] );
		}
		VCIL_LOG_V ( "\r\n");
	}
#endif
    
    return SUCCESS;
}

int VCILPort::Send_packet ( WPACKET *packet )
{
/*
#define ESCAPE_ADD(data) \
	if(data == SYNC) { \
		buf[len++]= ESCAPE; \
		buf[len++]= ESCAPE_SEQUENCE1; \
		checksum+= SYNC; \
	} else if(data == ESCAPE) { \
		buf[len++]= ESCAPE; \
		buf[len++]= ESCAPE_SEQUENCE2; \
		checksum+=ESCAPE; \
	} else { \
		buf[len++]= data; \
		checksum+= data; \
	}
	
	unsigned char buf[SEND_MSG_MAX_SIZE];
	unsigned char checksum = 0;
	int len = 0;
	
	buf[len++]= SYNC;
	ESCAPE_ADD( (packet->length >> 8) );
	ESCAPE_ADD( (packet->length & 0xFF) );
	ESCAPE_ADD( (packet->cmd) );
	
	for(int i=0; i< packet->payload_length; i++)
	{
		ESCAPE_ADD( (packet->payload[i]) );
	}
	
	//printf("wait 2 = %d - %d\n", len, packet->payload_length);
	
	checksum = ~checksum + 1;
	ESCAPE_ADD( checksum );
	
	while ((wbuf_max_size - wbuf_size ) < len) 
	{
		// sleep?
	}
		
	pthread_mutex_lock(&wbuf_mutex);
			
	if( wbuf_rear <=  wbuf_front )
	{
		memcpy( &write_buffer[wbuf_rear], buf, len);
		wbuf_rear += len;
	}
	else
	{
		if( wbuf_rear + len > wbuf_max_size)
		{
			int writebyte = wbuf_max_size - wbuf_rear;
			memcpy( &write_buffer[wbuf_rear], buf, writebyte);
			memcpy( &write_buffer[0], &buf[writebyte], len-writebyte);
			wbuf_rear = len - writebyte;
		}
		else
		{
			memcpy( &write_buffer[wbuf_rear], buf, len);
			wbuf_rear+=len;
		}		
	}
	
	wbuf_size+=len;
		
	pthread_mutex_unlock(&wbuf_mutex);

	return SUCCESS;
	*/
	return Send_packet_no_wait(packet);
}

int VCILPort::Send_packet_no_wait ( WPACKET *packet )
{
#define ESCAPE_ADD(data) \
	if(data == SYNC) { \
		buf[len++]= ESCAPE; \
		buf[len++]= ESCAPE_SEQUENCE1; \
		checksum+= SYNC; \
	} else if(data == ESCAPE) { \
		buf[len++]= ESCAPE; \
		buf[len++]= ESCAPE_SEQUENCE2; \
		checksum+=ESCAPE; \
	} else { \
		buf[len++]= data; \
		checksum+= data; \
	}
		
	unsigned char buf[SEND_MSG_MAX_SIZE];
	unsigned char checksum = 0;
	int len = 0;
	
	buf[len++]= SYNC;
	ESCAPE_ADD( (packet->length >> 8) );
	ESCAPE_ADD( (packet->length & 0xFF) );
	ESCAPE_ADD( (packet->cmd) );

	for(int i=0; i< packet->payload_length; i++)
	{
		ESCAPE_ADD( (packet->payload[i]) );
	}
	
	checksum = ~checksum + 1;
	ESCAPE_ADD( checksum );
	
	Send( (unsigned char *)buf, len);
			
	return SUCCESS;
}

int VCILPort::Recv_packet ( RPACKET *packet, unsigned long timeout )
{
	static int checksum_fail_count = 0;
	int ret;
	unsigned char LRC;
	unsigned char checksum = 0;
	unsigned char data[1];
	unsigned short len = 0;		//	The length of PAYLOAD
	unsigned char IsGotSYNC = FALSE;
	unsigned char IsGotESCAPE = FALSE;
	unsigned char IsGotLENGTH_MSB = FALSE;
	unsigned char IsGotLENGTH_LSB = FALSE;
	unsigned char IsGotCMD = FALSE;
	unsigned long tick = GET_TICK_COUNT;
	
	for ( int i = 0 ; ( GET_TICK_COUNT - tick ) < timeout ; ++i )
	{
		//ret = recv ( data, i, 1, 500 );
		//ret = RecvByte ( data[0], 100);
		ret = RecvByte ( data[0], 5);
		if ( ret != SUCCESS )
		{
			if( ret > 2 )
			{
				VCIL_LOG_E("err=%d\n", ret);
			}
			continue;
		}
		//else
			//VCIL_LOG_V ( "%02X ",  data[0]);

		tick = GetTickCount ( );
		
		if ( ( IsGotSYNC == FALSE ) && ( data[0] == SYNC ) )
		{
#ifdef DEBUG_RAW_DATA
			fprintf(debug_raw_file, "\b \n%02X,", data[0]);
#endif
			packet->sync = data[0];
			IsGotSYNC = TRUE;
			IsGotESCAPE = FALSE;
			IsGotLENGTH_MSB = FALSE;
			IsGotLENGTH_LSB = FALSE;
			IsGotCMD = FALSE;
			len = 0;
			checksum = 0;
			continue;
		}
#ifdef DEBUG_RAW_DATA
		fprintf(debug_raw_file, "%02X,", data[0]);
#endif

		if ( IsGotSYNC == FALSE )
			continue;

		if ( ( IsGotESCAPE == FALSE ) && ( data[0] == ESCAPE ) )
		{
			IsGotESCAPE = TRUE;
			continue;
		}

		//	The default ESCAPE code is 0xDB
		if ( IsGotESCAPE == TRUE )
		{
			switch(data[0])
			{
				case ESCAPE_SEQUENCE1:
				{
					data[0] = ESCAPE_SEQUENCE1_ORIGINAL;
				}
				break;
				case ESCAPE_SEQUENCE2:
				{
					data[0] = ESCAPE_SEQUENCE2_ORIGINAL;				
				}
				break;
				default:
				return ERR_DATA_FORMAT;
				break;
			}

			IsGotESCAPE = FALSE;
		}

		//	Transfer completed
		//	Response communications protocol definition
		//	|  SYNC  |  Length   |  CMD   |  PAYLOAD    | Checksum |
		//	|		 |			 |        |             |          |
		//	| 1 byte |  2 bytes  | 1 byte | 0 ~ N bytes |  1 byte  |
		if ( IsGotLENGTH_MSB == FALSE )
		{
			packet->length = data[0];
			checksum+=data[0];
			IsGotLENGTH_MSB = TRUE;
			continue;
		}

		if ( IsGotLENGTH_LSB == FALSE )
		{
			( packet->length <<= 8 ) |= data[0];
			checksum+=data[0];
			IsGotLENGTH_LSB = TRUE;

			if ( packet->length - 2 >= (unsigned short) packet->payload_length )
			{
				return ERR_DATA_BUFFER_OVERFLOW;
			}

			continue;
		}

		if ( IsGotCMD == FALSE )
		{
			packet->cmd = data[0];
			checksum+=data[0];
			IsGotCMD = TRUE;
			continue;
		}
		
		//	Pasing the PAYLOAD field, not including the CMD and CHECKSUM fields
		if ( len < packet->length - 2 )
		{
			packet->payload[len++] = data[0];
			checksum+=data[0];
			continue;
		}

		checksum+=data[0];
		if ( checksum != 0 )
		{
			//VCIL_LOG_V("%d CK fail %02X != %02X\n", checksum_fail_count++, data[0] , checksum);
			//printf("%d CK fail %02X != %02X\n", checksum_fail_count++, data[0] , checksum);
#ifdef DEBUG_RAW_DATA
			fprintf(debug_raw_file, "CKE\n");
			fflush(debug_raw_file);
#endif
			return ERR_LRC_FIELD;
		}

		packet->checksum = data[0];

		if( VCIL_DEBUG_LOG_FLAG_VMSG & debug_level )
		{
			DEBUG_PRINT("[%d]RX[%hu]>%02X,",port_id, packet->length, packet->cmd);
			for(int i =0;i<packet->length-2;i++)
			{
				DEBUG_PRINT("%02X,", packet->payload[i]);
			}
			DEBUG_PRINT("%02X\n", LRC);
			
			if( checksum_fail_count > 0)
				VCIL_LOG_V("checksum fail %d times\n", checksum_fail_count);
			
		}
				
		return SUCCESS;
	}

	return ERR_TIMEOUT;
}

