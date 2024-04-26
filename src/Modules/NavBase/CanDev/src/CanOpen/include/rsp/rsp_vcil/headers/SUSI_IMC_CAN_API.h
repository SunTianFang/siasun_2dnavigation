#ifndef _SUSI_IMC_CAN_API_H
#define _SUSI_IMC_CAN_API_H

DLLAPI USHORT SUSI_IMC_CAN_Read ( IN UINT port, OUT PIMC_CAN_MSG_OBJECT object );
DLLAPI USHORT SUSI_IMC_CAN_ReadTimedWait ( IN UINT port, OUT PIMC_CAN_MSG_OBJECT object , unsigned int ms);
DLLAPI USHORT SUSI_IMC_CAN_Write ( IN UINT port, IN PIMC_CAN_MSG_OBJECT object );
DLLAPI USHORT SUSI_IMC_CAN_GetBusErrorStatus ( IN UINT port, OUT PIMC_CAN_ERROR_STATUS_OBJECT object );
DLLAPI USHORT SUSI_IMC_CAN_SetEvent ( IN UINT port, void* hEvent );
DLLAPI USHORT SUSI_IMC_CAN_SetBitTiming ( IN UINT port, IN CAN_SPEED bit_rate );
DLLAPI USHORT SUSI_IMC_CAN_GetBitTiming ( IN UINT port, OUT CAN_SPEED *bit_rate, OUT CAN_BUS_MODE *mode );
DLLAPI USHORT SUSI_IMC_CAN_SetBitTimingSilence ( IN UINT port, IN CAN_SPEED bit_rate );
DLLAPI USHORT SUSI_IMC_CAN_SetMessageMask ( IN UINT port, IN PIMC_CAN_MASK_OBJECT mask_object);
DLLAPI USHORT SUSI_IMC_CAN_GetMessageMask ( IN UINT port, OUT PIMC_CAN_MASK_OBJECT pmask_object);
DLLAPI USHORT SUSI_IMC_CAN_RemoveMessageMask ( IN UINT port, IN int mask_number );
DLLAPI USHORT SUSI_IMC_CAN_ResetMessageMask ( IN UINT port );

#endif
