//                                - RFID.H -
//
//   Implementation of class "CRfId".
//
//   Author: Zhanglei
//   Date:   2004. 5. 25
//

#ifndef _CRfId_
#define _CRfId_

#include "ZTypes.h"

#ifndef _MFC_VER
  #include "Archive.h"
#endif

#define RFID_CODE_LEN               6

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CRfId".
class DllExport CRfId
{
public:
	UCHAR m_uchCode[RFID_CODE_LEN];

public:
	CRfId();
	CRfId(UCHAR *pBuf);

	void Init(UCHAR* pBuf);
	bool operator == (CRfId& Obj);
	bool operator != (CRfId& Obj);
	bool Usable();

	// Create the node base from a text file
	bool Create(FILE *StreamIn);

	// Save the node base to a text file
	bool Save(FILE *StreamOut);

	// Archive I/O routine
	friend DllExport CArchive& operator >> (CArchive& ar, CRfId& Obj);
	friend DllExport CArchive& operator << (CArchive& ar, CRfId& Obj);
};
#endif
