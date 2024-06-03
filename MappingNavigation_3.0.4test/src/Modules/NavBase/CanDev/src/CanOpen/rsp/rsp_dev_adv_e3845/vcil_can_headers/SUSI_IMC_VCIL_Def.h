#ifndef _SUSI_VCIL_DEF_H
#define _SUSI_VCIL_DEF_H

#define VCIL_FIRMWARE_LENGTH 18

enum VCIL_CAN_CHANNEL_MODE
{
	VCIL_MODE_CAN                 = 0x00,
	VCIL_MODE_J1939,
	VCIL_MODE_OBD2
};
typedef int* PVCIL_CAN_CHANNEL_MODE;

#endif
