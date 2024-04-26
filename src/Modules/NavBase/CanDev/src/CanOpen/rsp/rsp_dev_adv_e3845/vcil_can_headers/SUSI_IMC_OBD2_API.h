#ifndef _SUSI_IMC_OBD2_API_H
#define _SUSI_IMC_OBD2_API_H

DLLAPI USHORT SUSI_IMC_OBD2_Read( IN UINT port, OUT PIMC_OBD2_MSG_OBJECT object );
DLLAPI USHORT SUSI_IMC_OBD2_Write( IN UINT port, IN PIMC_OBD2_MSG_OBJECT object );
DLLAPI USHORT SUSI_IMC_OBD2_SetEvent( IN UINT port, void* hEvent );
DLLAPI USHORT SUSI_IMC_OBD2_AddMessageFilter(IN UINT port,  UINT pid );
DLLAPI USHORT SUSI_IMC_OBD2_GetMessageFilter(IN UINT port,  UINT* total, UINT* pid );
DLLAPI USHORT SUSI_IMC_OBD2_RemoveMessageFilter( IN UINT port,  IN UINT pid );
DLLAPI USHORT SUSI_IMC_OBD2_RemoveAllMessageFilter ( IN UINT port );

#endif
