#ifndef _VCIL_PORT_H
#define _VCIL_PORT_H

#include "SUSI_IMC.h"
#include "common.h"
#include <pthread.h>
#include <stdio.h>

#define RECV_BUFFER_SIZE 16

class VCILPort
{
public:
	VCILPort();
	~VCILPort();
	
	BOOL SetBaudRate(int hCOMPort, DWORD dwBaudRate);
	USHORT Open();
	USHORT Open(const char *path, int init_baudrate);
	void Close();
	void Purge();
	
	int Send_packet_no_wait ( WPACKET *packet );
	int Send_packet ( WPACKET *packet );
	int Recv_packet ( RPACKET *packet, unsigned long timeout );
	
private:
	bool is_open;
	int vcil_device_port;
	int port_id;
	
	int baudrate;
	char node_path[256];
	
	BYTE recv_buffer[RECV_BUFFER_SIZE];
	
	//int recv_size = 0;
	//int recv_index = 0;
	int recv_size;
	int recv_index;

	static const int wbuf_max_size = 256;
	char write_buffer[wbuf_max_size];
	int wbuf_front;
	int wbuf_rear;
	int wbuf_size;
	pthread_mutex_t wbuf_mutex;
	
	int RecvByte ( unsigned char &b,unsigned int timeout);
	int Send ( unsigned char *buf, unsigned long len );
	
	FILE *debug_raw_file;
	
	void SendData();
};

#endif
