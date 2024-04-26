#ifndef _SUSI_IMC_VCIL_API_H
#define _SUSI_IMC_VCIL_API_H

DLLAPI USHORT SUSI_IMC_VCIL_Initialize ();
DLLAPI USHORT SUSI_IMC_VCIL_Deinitialize ();
DLLAPI USHORT SUSI_IMC_VCIL_GetLibVersion ( OUT char *version );
DLLAPI USHORT SUSI_IMC_VCIL_GetFWVersion ( OUT char* version );
DLLAPI USHORT SUSI_IMC_VCIL_ResetModule ( void );
DLLAPI USHORT SUSI_IMC_VCIL_ModuleControl ( IN int port1, IN int port2);

#endif
