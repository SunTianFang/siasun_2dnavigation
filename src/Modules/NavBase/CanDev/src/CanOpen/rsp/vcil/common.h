
#ifndef _COMMON_H
#define _COMMON_H

#define VCIL_LIB_VERSION	"3.8.0"
#define CreateEvent(x,...) PTHREAD_COND_INITIALIZER
typedef void* CRITICAL_SECTION;


#define CAN_PORT1							0
#define CAN_PORT2							1

//	Error code set of library
#define ERR_INVALID_ARGUMENT                10
#define ERR_CMD_NOT_MATCH					9
#define ERR_ACK_FAIL						8
#define ERR_READ_PORT						7
#define ERR_WRITE_PORT						6
#define ERR_DATA_BUFFER_OVERFLOW			5
#define ERR_LRC_FIELD						4
#define ERR_DATA_FORMAT						3
#define ERR_TIMEOUT							2
#define ERR_EMPTY_DATA						1
#define SUCCESS								0
#define ERR_INVALID_FILE_PATH				(-1)

//	It depends on OS platform
#define TIMEOUT_MS							10
#define GET_TICK_COUNT                      GetTickCount ( )

//	Command ID list
//	for CAN Message Class
#define CMD_VMSG_TX_CAN						0x14
#define CMD_VMSG_TX_CAN_EX					0x5F
#define CMD_VMSG_RX_CAN						0x15
#define CMD_VMSG_RX_CAN_EX					0x5E
#define CMD_VMSG_POLL_CAN					0x60
#define CMD_VMSG_READ_FILTER_STATUS_CAN		0xAE
#define CMD_VMSG_READ_FILTER_STATUS_CAN_RESP 0xAF
#define CMD_VMSG_BITRATE_CAN				0x50
#define CMD_VMSG_BITRATE_CAN_BTR			0x54

#define CMD_VMSG_READ_CAN_ESR               0x5B
#define CMD_VMSG_READ_CAN_ESR_RESP          0x5C
#define CMD_VMSG_READ_BITRATE_CAN           0x56
#define CMD_VMSG_READ_BITRATE_CAN_RESP      0x55

#define CMD_VMSG_CLEAR_FILTER_LIST_CAN		0xA0
#define CMD_VMSG_ADD_FILTER_LIST_CAN		0xA1
#define CMD_VMSG_REMOVE_FILTER_LIST_CAN		0xA2
#define CMD_VMSG_CLEAR_FILTER_MASK_CAN		0xA4
#define CMD_VMSG_ADD_FILTER_MASK_CAN_MODE_A	0xA5
#define CMD_VMSG_ADD_FILTER_MASK_CAN_MODE_B	0xA6
#define CMD_VMSG_READ_FILTER_MASK_CAN		0xFC
#define CMD_VMSG_READ_FILTER_MASK_CAN_RESP	0xFD
#define CMD_VMSG_REMOVE_FILTER_MASK_CAN		0xA7
#define CMD_VMSG_POLL_CAN_STATISTICS        0x63
#define CMD_VMSG_POLL_CAN_STATISTICS_RESP   0x17

//	for J1708 Message Class
#define CMD_VMSG_TX_J1708                    0x08
#define CMD_VMSG_RX_J1708                    0x16
#define CMD_VMSG_POLL_J1708                  0x61
#define CMD_VMSG_POLL_J1708_STATISTICS       0x62
#define CMD_VMSG_POLL_J1708_STATISTICS_RESP  0xB0
#define CMD_VMSG_STATISTICS_J1708            0xB1
#define CMD_VMSG_CLEAR_FILTER_LIST_J1708     0xA3
#define CMD_VMSG_ADD_FILTER_LIST_J1708       0xAC
#define CMD_VMSG_REMOVE_FILTER_LIST_J1708    0xAD
#define CMD_VMSG_READ_FILTER_LIST_J1708      0xFE
#define CMD_VMSG_READ_FILTER_LIST_J1708_RESP 0xE2

// for J1939 Message Class
#define CMD_VMSG_TX_J1939                    0x05
#define CMD_VMSG_RX_J1939                    0x06
#define CMD_VMSG_PERIODIC_TX_J1939           0x07
#define CMD_VMSG_STATISTICS_J1939            0xB3
#define CMD_VMSG_CLEAR_FILTER_LIST_J1939	 0xE9
#define CMD_VMSG_ADD_FILTER_LIST_J1939       0x01
#define CMD_VMSG_REMOVE_FILTER_LIST_J1939    0x02
#define CMD_VMSG_READ_FILTER_LIST_J1939      0xE3
#define CMD_VMSG_READ_FILTER_LIST_J1939_RESP 0xE4
#define CMD_VMSG_SET_ADDRESS_AND_NAME_J1939  0xB5
#define CMD_VMSG_GET_ADDRESS_AND_NAME_J1939  0xB6
#define CMD_VMSG_GET_ADDRESS_AND_NAME_J1939_RESP  0xB7

// for OBD2 Message Class
#define CMD_VMSG_TX_OBD2                    0x2A
#define CMD_VMSG_RX_OBD2                    0x2B
#define CMD_VMSG_PERIODIC_TX_OBD2           0x2C           
#define CMD_VMSG_STATISTICS_OBD2            0xB4
#define CMD_VMSG_CLEAR_FILTER_LIST_OBD2     0xEA
#define CMD_VMSG_ADD_FILTER_LIST_OBD2       0x28
#define CMD_VMSG_REMOVE_FILTER_LIST_OBD2    0x29
#define CMD_VMSG_READ_FILTER_LIST_OBD2      0xE5
#define CMD_VMSG_READ_FILTER_LIST_OBD2_RESP 0xE6

// for J1587 Message Class
#define CMD_VMSG_TX_J1587                    0x10
#define CMD_VMSG_RX_J1587                    0x09
#define CMD_VMSG_PERIODIC_TX_J1587           0xAB
#define CMD_VMSG_STATISTICS_J1587            0xB2
#define CMD_VMSG_CLEAR_FILTER_LIST_J1587     0xEB
#define CMD_VMSG_ADD_FILTER_LIST_J1587       0x03
#define CMD_VMSG_REMOVE_FILTER_LIST_J1587    0x04
#define CMD_VMSG_READ_FILTER_LIST_J1587      0xE7
#define CMD_VMSG_READ_FILTER_LIST_J1587_RESP 0xE8

//	for Maintain Class
#define CMD_VMSG_SIMULATE_CAN				0x51
#define CMD_VMSG_MSG_GENERATE_CAN           0x52

//	for CONFIG Class
#define CMD_VMSG_BAUDRATE_MODULE			0x53
#define CMD_VMSG_BAUDRATE_J1708				0x51
#define CMD_VMSG_FIRMWARE_VERSION			0xF1
#define CMD_VMSG_FIRMWARE_VERSION_RESP		0xF8
#define CMD_VMSG_MODE_RECEIVE				0xF0
#define CMD_VMSG_MODULE_CONTROL				0xF2
#define CMD_VMSG_RESET_MODULE				0x11
#define CMD_VMSG_ENTER_BOOTLOADER			0xF3
#define CMD_VMSG_PLATFORM_NAME				0xF4
#define CMD_VMSG_GET_BOOTLOADER_CHECKSUM	0xF6

//	for Response CMD ID
#define ACK_VMSG_CAN						0x00
#define ACK_VMSG_J1708						0x00
#define ACK_VMSG_PLATFORM_NAME				0xF5
#define ACK_VMSG_BOOTLOADER_CHECKSUM		0xF7
#define MRES_POLL_EMPTY                     0xE0

//	Escape sequence sets
#define	SYNC								0xC0
#define ESCAPE								0xDB
#define ESCAPE_SEQUENCE1					0xDC
#define ESCAPE_SEQUENCE1_ORIGINAL			0xC0		//	0xC0 -> 0xDB 0xDB
#define ESCAPE_SEQUENCE2					0xDD
#define ESCAPE_SEQUENCE2_ORIGINAL			0xDB		//	0xDB -> 0xDB 0xDD
#define CAN_DATA_MAX_SIZE					8
#define CAN_MSG_MAX_SIZE					32
#define J1708_MSG_MAX_SIZE					32
#define SEND_MSG_MAX_SIZE					512
#define PLATFORM_NAME_MAX_SIZE				12

//	for vmsg_bitrate_can()
#define BITS_1M								0x00
#define BITS_800K							0x01
#define BITS_500K							0x02
#define BITS_250K							0x03
#define BITS_200K							0x04
#define BITS_125K							0x05
#define BITS_50K							0x06
#define BITS_20K							0x07
#define BITS_10K							0x08
#define BITS_5K								0x09

//	for vmsg_clear_filter_list_can()
#define VMSG_CLEAR_FILTER_LIST_CAN_UNLOCK_KEY	"\x43\69\x34"

//	for vmsg_clear_filter_mask_can()
#define VMSG_CLEAR_FILTER_MASK_CAN_UNLOCK_KEY	"\x43\69\x35"

//	for vmsg_clear_filter_list_j1708()
#define VMSG_CLEAR_FILTER_LIST_J1708_UNLOCK_KEY	"\x43\69\x34"

//	for vmsg_sumulate_can()
#define TURN_OFF_SIMULATION								0x00
#define GEN_STD_CAN1_UNIFY_MSG_AND_LENGTH				0x01
#define GEN_STD_CAN1_UNIFY_MSG_BUT_RANDOM_LENGTH		0x02
#define GEN_STD_CAN1_PATTERN_MSG_BUT_RANDOM_LENGTH		0x03
#define GEN_STD_CAN2_UNIFY_MSG_AND_LENGTH				0x04
#define GEN_STD_CAN2_UNIFY_MSG_BUT_RANDOM_LENGTH		0x05
#define GEN_STD_CAN2_PATTERN_MSG_BUT_RANDOM_LENGTH		0x06
#define GEN_EXT_CAN1_UNIFY_MSG_AND_LENGTH				0x11
#define GEN_EXT_CAN1_UNIFY_MSG_BUT_RANDOM_LENGTH		0x12
#define GEN_EXT_CAN1_PATTERN_MSG_BUT_RANDOM_LENGTH		0x13
#define GEN_EXT_CAN2_UNIFY_MSG_AND_LENGTH				0x14
#define GEN_EXT_CAN2_UNIFY_MSG_BUT_RANDOM_LENGTH		0x15
#define GEN_EXT_CAN2_PATTERN_MSG_BUT_RANDOM_LENGTH		0x16
#define GEN_STD_BOTH_UNIFY_MSG_AND_LENGTH				0x20
#define GEN_EXT_BOTH_UNIFY_MSG_AND_LENGTH				0x21
#define GEN_EXT_BOTH_RANDOM_MSG_AND_RANDOM_LENGTH		0x22

//	for vmg_baudrate_j1708()
#define BAUDRATE_J1708_9600					0x00
#define BAUDRATE_J1708_14400				0x01

//  for vmsg_baudrate_module
#define BAUDRATE_MODULE_9600                0x00
#define BAUDRATE_MODULE_14400               0x01
#define BAUDRATE_MODULE_19200               0x02
#define BAUDRATE_MODULE_38400               0x03
#define BAUDRATE_MODULE_57600               0x04
#define BAUDRATE_MODULE_115200              0x05
#define BAUDRATE_MODULE_230400              0x06
#define BAUDRATE_MODULE_460800              0x07
#define BAUDRATE_MODULE_921600              0x08
#define BAUDRATE_MODULE_1843200             0x09
#define BAUDRATE_MODULE_3686400             0x0A
#define BAUDRATE_MODULE_1500000             0x0B

//	for vmsg_mode_receive()
#define CAN_ASYNC_MODE						(0)
#define CAN_POLLING_MODE					(1)
#define J1708_ASYNC_MODE					(0)
#define J1708_POLLING_MODE					(1<<1)

//	for vmsg_reset_module()
#define VMSG_RESET_MODULE_UNLOCK_KEY		"\x5A\x69\xA5"

//	for vmsg_enter_bootloader()
#define VMSG_ENTER_BOOTLOADER_UNLOCK_KEY	"\x43\x69\x34"

#define CAN_MODE_A          0
#define CAN_MODE_B          1

#define MAX_MASK_NUMBER     14

#define MODULE_CONTROL_CAN     1 << 0;
#define MODULE_CONTROL_J1708   1 << 1;
#define MODULE_CONTROL_J1939   1 << 2;
#define MODULE_CONTROL_OBD2    1 << 3;
#define MODULE_CONTROL_J1587   1 << 4;

#define CHANNEL_MODE_CAN                 0x00
#define CHANNEL_MODE_CAN_WITH_FILTER     0x01
#define CHANNEL_MODE_J1939               0x02
#define CHANNEL_MODE_J1939_WITH_FILTER   0x03
#define CHANNEL_MODE_OBD2                0x04
#define CHANNEL_MODE_OBD2_WITH_FILTER    0x05

//	For request/response commands of TREK-110
#pragma pack ( push )			/* Push current alignment to stack */
#pragma pack ( 1 )				/* Set alignment to 1 byte */
typedef struct _RPACKET
{
	unsigned char sync;
	union
	{
		unsigned short length;
		// Avoid little - endian, it not fit protocol definition ( MSB first )
		unsigned char blen[2];
	};
	unsigned char cmd;
	unsigned char *payload;
	unsigned payload_length;
	unsigned char checksum;

} RPACKET, WPACKET;
#pragma pack ( pop )			 /* Restore original alignment from stack */

extern void __put_unaligned_2_be ( unsigned long __v, unsigned char *__p );
extern void __put_unaligned_4_be ( unsigned long __v, unsigned char *__p );
extern void init_WRPACKET ( RPACKET *x, unsigned char cmd, unsigned char *payload, unsigned payload_length );
extern void init_RRPACKET ( RPACKET *x, unsigned char *payload, unsigned payload_length );
extern double GetTickCount(void);

#endif
