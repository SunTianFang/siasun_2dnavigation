#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>

#include "common.h"
#include "DebugLog.h"
#include "vcil_vmsg.h"

#define CAN_MASK_ENABLE                      0x01
#define CAN_MASK_DISABLE                     0x00

#define J1939_DATA_PACKET_SIZE_CHANNEL       0x01
#define J1939_DATA_PACKET_SIZE_PGN           0x03
#define J1939_DATA_PACKET_SIZE_DST           0x01
#define J1939_DATA_PACKET_SIZE_SRC           0x01
#define J1939_DATA_PACKET_SIZE_PRI           0x01
#define J1939_DATA_PACKET_DATA_PAYLOAD_SIZE J1939_DATA_PACKET_SIZE_CHANNEL + J1939_DATA_PACKET_SIZE_PGN + J1939_DATA_PACKET_SIZE_DST + \
	                                        J1939_DATA_PACKET_SIZE_SRC + J1939_DATA_PACKET_SIZE_PRI

#define OBD2_DATA_PACKET_SIZE_CHANNEL        0x01
#define OBD2_DATA_PACKET_SIZE_DST            0x01
#define OBD2_DATA_PACKET_SIZE_SRC            0x01
#define OBD2_DATA_PACKET_SIZE_PRI            0x01
#define OBD2_DATA_PACKET_SIZE_TAT            0x01
#define OBD2_DATA_PACKET_DATA_PAYLOAD_SIZE   OBD2_DATA_PACKET_SIZE_CHANNEL + OBD2_DATA_PACKET_SIZE_DST + OBD2_DATA_PACKET_SIZE_SRC + \
                                             OBD2_DATA_PACKET_SIZE_PRI + OBD2_DATA_PACKET_SIZE_TAT
#define OBD2_DATA_PACKET_SIZE_PID            0x01

VMSG::VMSG(VCILPort *p)
:port(p)
{
}

VMSG::~VMSG()
{
}

int VMSG::vmsg_firmware_version ( )
{
	int ret;
	RPACKET cfg_packet;

	VCIL_LOG_V ( "vmsg_firmware_version Start \r\n" );

	init_WRPACKET ( &cfg_packet, CMD_VMSG_FIRMWARE_VERSION, NULL, 0 );

	ret = port->Send_packet_no_wait ( &cfg_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d \r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_firmware_version Finish \r\n");

	return SUCCESS;
}

int VMSG::vmsg_baudrate_module ( unsigned char baudrate )
{
	int ret;
	RPACKET cfg_packet;
	unsigned char wpayload[2];

	VCIL_LOG_V ( "vmsg_baudrate_module Start \r\n" );

	init_WRPACKET ( &cfg_packet, CMD_VMSG_BAUDRATE_MODULE, wpayload, sizeof ( wpayload ) );
	wpayload[0] = baudrate;
	wpayload[1] = 0x00;

	//ret = port->Send_packet ( &cfg_packet );
	ret = port->Send_packet_no_wait ( &cfg_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_baudrate_module Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_baudrate_module Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_mode_receive ( unsigned char mode )
{
	int ret;
	RPACKET cfg_packet;
	unsigned char wpayload[1];

	VCIL_LOG_V ( "vmsg_mode_receive Start \r\n" );

	init_WRPACKET ( &cfg_packet, CMD_VMSG_MODE_RECEIVE, wpayload, sizeof ( wpayload ) );
	wpayload[0] = mode;
    VCIL_LOG_V ( "vmsg_mode_receive port->Send_packet ... \r\n" );
	ret = port->Send_packet_no_wait ( &cfg_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_mode_receive Fails to start, ret = %d\r\n", ret );
		return ret;
	}
    VCIL_LOG_V ( "vmsg_mode_receive port->Send_packet ... ok !\r\n" );
	VCIL_LOG_V ( "vmsg_mode_receive Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_module_control ( unsigned char can_channel1_protocol_select)
{
	int ret;
	RPACKET cfg_packet;
	unsigned char wpayload[4] = {'\0'};

	VCIL_LOG_V ( "vmsg_module_control Start \r\n" );

	init_WRPACKET ( &cfg_packet, CMD_VMSG_MODULE_CONTROL, wpayload, sizeof ( wpayload ) );
	wpayload[0] = can_channel1_protocol_select;
	wpayload[1] = 0xFF; // Disable Port
	wpayload[2] = 0xFF; // Disable Port
	wpayload[3] = 0xFF; // Disable Port

	ret = port->Send_packet ( &cfg_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_module_control Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_reset_module ( void )
{
	int ret;
	RPACKET cfg_packet;
	const char *wpayload = VMSG_RESET_MODULE_UNLOCK_KEY;

	VCIL_LOG_V ( "vmsg_reset_module Start \r\n" );

	init_WRPACKET ( &cfg_packet, CMD_VMSG_RESET_MODULE, ( unsigned char * ) wpayload, strlen ( wpayload ) );

	ret = port->Send_packet_no_wait ( &cfg_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_reset_module Finish \r\n" );
	
	port->Close();			
	usleep(1500000);
	port->Open();
	port->Purge();

	vmsg_mode_receive(0);

	return SUCCESS;
}

int VMSG::vmsg_poll_can ()
{
	int ret;
	RPACKET can_packet;
	
	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &can_packet, CMD_VMSG_POLL_CAN, NULL, 0 );

	ret = port->Send_packet_no_wait ( &can_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_tx_can ( unsigned char can_port, unsigned long can_id, unsigned char data_length, unsigned char *data )
{
	int ret;
	RPACKET can_packet;
	unsigned char rpayload[CAN_MSG_MAX_SIZE];
	unsigned long idx = 0;

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &can_packet, CMD_VMSG_TX_CAN, rpayload, 1 + 4 + data_length );

	rpayload[idx++] = can_port;
	__put_unaligned_4_be ( can_id, rpayload + idx );
	idx += 4;
	memcpy ( rpayload + idx, data, data_length );

	ret = port->Send_packet_no_wait ( &can_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_tx_can_old ( unsigned char can_port, unsigned char rtr, unsigned long can_id, unsigned char data_length, unsigned char *data )
{
	int ret;
	RPACKET can_packet;
	unsigned char rpayload[CAN_MSG_MAX_SIZE];
	unsigned long idx = 0;

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &can_packet, CMD_VMSG_TX_CAN, rpayload, 2 + 4 + data_length );

	rpayload[idx++] = can_port;
	rpayload[idx++] = rtr;
	__put_unaligned_4_be ( can_id, rpayload + idx );
	idx += 4;
	memcpy ( rpayload + idx, data, data_length );

	ret = port->Send_packet_no_wait ( &can_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_tx_can_ex ( unsigned char can_port, unsigned char rtr, unsigned long can_id, unsigned char data_length, unsigned char *data )
{
	int ret;
	RPACKET can_packet;
	unsigned char rpayload[CAN_MSG_MAX_SIZE];
	unsigned long idx = 0;

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &can_packet, CMD_VMSG_TX_CAN_EX, rpayload, 2 + 4 + data_length );

	rpayload[idx++] = can_port;
	rpayload[idx++] = rtr;
	__put_unaligned_4_be ( can_id, rpayload + idx );
	idx += 4;
	memcpy ( rpayload + idx, data, data_length );

	ret = port->Send_packet_no_wait ( &can_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_bitrate_can ( unsigned char can_port, unsigned char bitrate_flag, int mode )
{
	int ret;
	RPACKET bitrate_packet;
	unsigned char wpayload[3];

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &bitrate_packet, CMD_VMSG_BITRATE_CAN, wpayload, sizeof ( wpayload ) );

	wpayload[0] = can_port ;
	wpayload[1] = bitrate_flag;
	wpayload[2] = mode;


	ret = port->Send_packet ( &bitrate_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_bitrate_can_btr ( unsigned char can_port, unsigned char sjw, unsigned char bs1, unsigned char bs2, unsigned short prescaler, int mode )
{
	int ret;
	RPACKET bitrate_packet;
	unsigned char wpayload[7];

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &bitrate_packet, CMD_VMSG_BITRATE_CAN_BTR, wpayload, sizeof ( wpayload ) );

	wpayload[0] = can_port ;
	wpayload[1] = sjw;
	wpayload[2] = bs1;
	wpayload[3] = bs2;
	wpayload[4] = (prescaler >> 8)&0xFF;
	wpayload[5] = prescaler&0xFF;
	wpayload[6] = mode;

	ret = port->Send_packet ( &bitrate_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_read_can_esr ( unsigned char can_port )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload = can_port;

	VCIL_LOG_V ( "vmsg_read_can_esr Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_READ_CAN_ESR, &wpayload, sizeof ( wpayload ) );

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_read_can_esr Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_read_can_esr Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_read_can_bitrate ( unsigned char can_port )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload = can_port;

	VCIL_LOG_V ( "vmsg_read_can_bitrate Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_READ_BITRATE_CAN, &wpayload, sizeof ( wpayload ) );

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_read_can_bitrate Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_read_can_bitrate Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_clear_filter_mask_can ( unsigned char can_port )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload = can_port;

	VCIL_LOG_V ( "vmsg_clear_filter_mask_can Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_CLEAR_FILTER_MASK_CAN, &wpayload, sizeof ( wpayload ) );

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_clear_filter_mask_can Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_clear_filter_mask_can Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_add_filter_mask_can ( unsigned char can_port, unsigned char bank, unsigned char RTR, unsigned char mode, unsigned long filter_id, unsigned long mask )
{
	int ret;
	RPACKET filter_packet;

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	switch (mode)
	{
		case CAN_MODE_A: /* 2.0A */
			{
				unsigned char wpayload[12];
				init_WRPACKET ( &filter_packet, CMD_VMSG_ADD_FILTER_MASK_CAN_MODE_A, wpayload, sizeof ( wpayload ) );
				wpayload[0] = can_port;
				wpayload[1] = bank;
				wpayload[2] = RTR;
				wpayload[3] = CAN_MASK_DISABLE;

				__put_unaligned_2_be ( filter_id, wpayload + 4 );
				__put_unaligned_2_be ( mask, wpayload + 6 );

				wpayload[8] = 0x01;
				wpayload[9] = 0x01;
				wpayload[10] = 0x01;
				wpayload[11] = 0x01;

				ret = port->Send_packet ( &filter_packet );
				if ( ret != SUCCESS )
				{
					VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
					return ret;
				}
			}
			break;
		case CAN_MODE_B:
			{
				unsigned char wpayload[11];
				init_WRPACKET ( &filter_packet, CMD_VMSG_ADD_FILTER_MASK_CAN_MODE_B, wpayload, sizeof ( wpayload ) );
				wpayload[0] = can_port;
				wpayload[1] = bank;
				wpayload[2] = RTR;

				__put_unaligned_4_be ( filter_id, wpayload + 3 );
				__put_unaligned_4_be ( mask, wpayload + 7 );

				ret = port->Send_packet ( &filter_packet );
				if ( ret != SUCCESS )
				{
					VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
					return ret;
				}
			}
			break;
		default:
			return ERR_INVALID_ARGUMENT;

	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_read_filter_mask_can ( unsigned char can_port, unsigned char bank )
{
	int ret;
	RPACKET filter_packet;
    unsigned char wpayload[2];

	VCIL_LOG_V ( "vmsg_read_filter_mask_can Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_READ_FILTER_MASK_CAN, wpayload, sizeof(wpayload) );

    wpayload[0] = can_port;
    wpayload[1] = bank;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_read_filter_mask_can Fails to start, ret = %d\r\n" , ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_read_filter_mask_can Finish \r\n" );
	return SUCCESS;
}

int VMSG::vmsg_remove_filter_mask_can ( unsigned char can_port, unsigned char bank )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload[2];

	VCIL_LOG_V ( "%s Start \r\n", __FUNCTION__ );

	init_WRPACKET ( &filter_packet, CMD_VMSG_REMOVE_FILTER_MASK_CAN, wpayload, sizeof ( wpayload ) );
	wpayload[0] = can_port;
	wpayload[1] = bank;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		return ret;
	}

	VCIL_LOG_V ( "%s Finish \r\n", __FUNCTION__ );

	return SUCCESS;
}

int VMSG::vmsg_tx_j1939 ( unsigned char channel_number, unsigned int pgn, unsigned char dst, unsigned char src, unsigned pri, unsigned int data_length, unsigned char *data )
{
	int ret;
	int idx = 0;
	int total_packet_size = 0;
	RPACKET j1939_packet;
	unsigned char* wpayload;
	unsigned char* wpayload_idx;

	VCIL_LOG_V ( "vmsg_tx_j1939 Start \r\n" );

	total_packet_size = J1939_DATA_PACKET_DATA_PAYLOAD_SIZE + data_length;
	VCIL_LOG_V ( "vmsg_tx_j1939 total_packet_size = %d \r\n", total_packet_size);

	wpayload = (unsigned char*) malloc (total_packet_size);
	wpayload_idx = wpayload;

	init_WRPACKET ( &j1939_packet, CMD_VMSG_TX_J1939, wpayload, total_packet_size );

	*wpayload_idx++ = (unsigned char) channel_number;
	*wpayload_idx++ = (unsigned char) (pgn >> 16);
	*wpayload_idx++ = (unsigned char) (pgn >> 8);
	*wpayload_idx++ = (unsigned char) pgn;
	*wpayload_idx++ = (unsigned char) dst;
	*wpayload_idx++ = (unsigned char) src;
	*wpayload_idx++ = (unsigned char) pri;
	memcpy( wpayload_idx++, data, data_length) ;

	ret = port->Send_packet_no_wait ( &j1939_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "%s Fails to start, ret = %d\r\n", __FUNCTION__, ret );
		free(wpayload);
		return ret;
	}

	free(wpayload);

	VCIL_LOG_V ( "vmsg_tx_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_add_filter_list_j1939 ( unsigned char channel_number, unsigned int pgn )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload[J1939_DATA_PACKET_SIZE_CHANNEL + J1939_DATA_PACKET_SIZE_PGN];

	VCIL_LOG_V ( "vmsg_add_filter_list_j1939 Start \r\n" );
	VCIL_LOG_V ( "vmsg_add_filter_list_j1939 channel_number = %d \r\n",channel_number );
	VCIL_LOG_V ( "vmsg_add_filter_list_j1939 pgn = 0x%d \r\n",pgn );

	init_WRPACKET ( &filter_packet, CMD_VMSG_ADD_FILTER_LIST_J1939, wpayload, sizeof ( wpayload ) );
	wpayload[0] = (channel_number);
	wpayload[1] = (unsigned char ) (pgn >>16) ;
	wpayload[2] = (unsigned char ) (pgn >>8) ;
	wpayload[3] = (unsigned char ) pgn ;


	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_add_filter_list_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_add_filter_list_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_remove_filter_list_j1939 ( unsigned char channel_number, unsigned int pgn )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload[J1939_DATA_PACKET_SIZE_CHANNEL + J1939_DATA_PACKET_SIZE_PGN];

	VCIL_LOG_V ( "vmsg_remove_filter_list_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_REMOVE_FILTER_LIST_J1939, wpayload, sizeof ( wpayload ) );
	wpayload[0] = (channel_number);
	wpayload[1] = (unsigned char ) (pgn >>16) ;
	wpayload[2] = (unsigned char ) (pgn >>8) ;
	wpayload[3] = (unsigned char ) pgn ;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_remove_filter_list_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_remove_filter_list_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_read_filter_list_j1939 ( unsigned char channel )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload;

	VCIL_LOG_V ( "vmsg_read_filter_list_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_READ_FILTER_LIST_J1939, &wpayload, sizeof(wpayload) );
	wpayload = channel;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_read_filter_list_j1939 Fails to start, ret = %d\r\n" , ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_read_filter_list_j1939 Finish \r\n" );
	return SUCCESS;
}

int VMSG::vmsg_remove_all_filter_j1939( unsigned char channel )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload;

	VCIL_LOG_V ( "vmsg_remove_all_filter_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_CLEAR_FILTER_LIST_J1939, &wpayload, sizeof ( wpayload ) );
	wpayload = channel;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_remove_all_filter_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_remove_all_filter_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_set_address_and_name_j1939( unsigned char channel , unsigned char address_enabled, unsigned char source_address, unsigned char name_enalbed, unsigned char* source_name)
{
    int ret;
	RPACKET filter_packet;
	unsigned char wpayload[12];

	VCIL_LOG_V ( "vmsg_set_address_and_name_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_SET_ADDRESS_AND_NAME_J1939, wpayload, sizeof ( wpayload ) );
	wpayload[0] = channel;
    wpayload[1] = address_enabled;
    wpayload[2] = source_address;
    wpayload[3] = name_enalbed;

    memcpy(&wpayload[4], source_name, 8);

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_set_address_and_name_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_set_address_and_name_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_get_address_and_name_j1939( unsigned char channel)
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload;

	VCIL_LOG_V ( "vmsg_get_address_and_name_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_GET_ADDRESS_AND_NAME_J1939, &wpayload, sizeof ( wpayload ) );
	wpayload = channel;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_get_address_and_name_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_get_address_and_name_j1939 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_tx_obd2 ( unsigned char channel_number, unsigned char dst, unsigned char src, unsigned char pri, unsigned char tat, unsigned char data_length, unsigned char *data )
{
	int ret;
	int idx = 0;
	int total_packet_size = 0;
	RPACKET obd2_packet;
	unsigned char* wpayload;
	unsigned char* wpayload_idx;

	VCIL_LOG_V ( "vmsg_tx_obd2 Start \r\n" );

	total_packet_size = OBD2_DATA_PACKET_DATA_PAYLOAD_SIZE + data_length;
	VCIL_LOG_V ( "vmsg_tx_obd2 total_packet_size = %d \r\n", total_packet_size);

	wpayload = (unsigned char*) malloc (total_packet_size);
	wpayload_idx = wpayload;

	init_WRPACKET ( &obd2_packet, CMD_VMSG_TX_OBD2, wpayload, total_packet_size );

	*wpayload_idx++ = channel_number;
	*wpayload_idx++ = dst;
	*wpayload_idx++ = src;
	*wpayload_idx++ = pri;
	*wpayload_idx++ = tat;

	memcpy( wpayload_idx++, data, data_length) ;

	//for (int i = 0; i < total_packet_size ; i++)
	//{
	//	VCIL_LOG_V("0x%02x ", wpayload[i]);
	//}
	//VCIL_LOG_V("\r\n");

	ret = port->Send_packet_no_wait ( &obd2_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_tx_obd2 Fails to start, ret = %d\r\n", ret );
        free(wpayload);
		return ret;
	}

	free(wpayload);

	VCIL_LOG_V ( "vmsg_tx_obd2 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_add_filter_list_obd2 ( unsigned char channel_number, unsigned int pid )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload[OBD2_DATA_PACKET_SIZE_CHANNEL + OBD2_DATA_PACKET_SIZE_PID];

	VCIL_LOG_V ( "vmsg_add_filter_list_obd2 Start \r\n" );
	VCIL_LOG_V ( "vmsg_add_filter_list_obd2 channel_number = %d \r\n",channel_number );
	VCIL_LOG_V ( "vmsg_add_filter_list_obd2 pid = 0x%d \r\n",pid );

	init_WRPACKET ( &filter_packet, CMD_VMSG_ADD_FILTER_LIST_OBD2, wpayload, sizeof ( wpayload ) );
	wpayload[0] = channel_number;
	wpayload[1] = pid ;


	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_add_filter_list_obd2 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_add_filter_list_obd2 Finish \r\n" );

	return SUCCESS;
}

int VMSG::vmsg_remove_filter_list_obd2 ( unsigned char channel_number, unsigned char pid )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload[OBD2_DATA_PACKET_SIZE_CHANNEL + OBD2_DATA_PACKET_SIZE_PID];

	VCIL_LOG_V ( "vmsg_remove_filter_list_j1939 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_REMOVE_FILTER_LIST_OBD2, wpayload, sizeof ( wpayload ) );
	wpayload[0] = channel_number;
	wpayload[1] = pid;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_remove_filter_list_j1939 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_remove_filter_list_j1939 Finish \r\n" );

	return SUCCESS;
}
int VMSG::vmsg_read_filter_list_obd2 ( unsigned char channel )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload;

	VCIL_LOG_V ( "vmsg_read_filter_list_obd2 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_READ_FILTER_LIST_OBD2, &wpayload, sizeof(wpayload) );
	wpayload = channel;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_read_filter_list_obd2 Fails to start, ret = %d\r\n" , ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_read_filter_list_obd2 Finish \r\n" );
	return SUCCESS;
}
int VMSG::vmsg_remove_all_filter_obd2( unsigned char channel )
{
	int ret;
	RPACKET filter_packet;
	unsigned char wpayload;

	VCIL_LOG_V ( "vmsg_remove_all_filter_obd2 Start \r\n" );

	init_WRPACKET ( &filter_packet, CMD_VMSG_CLEAR_FILTER_LIST_OBD2, &wpayload, sizeof ( wpayload ) );
	wpayload = channel;

	ret = port->Send_packet ( &filter_packet );
	if ( ret != SUCCESS )
	{
		VCIL_LOG_E ( "vmsg_remove_all_filter_obd2 Fails to start, ret = %d\r\n", ret );
		return ret;
	}

	VCIL_LOG_V ( "vmsg_remove_all_filter_obd2 Finish \r\n" );

	return SUCCESS;
}
