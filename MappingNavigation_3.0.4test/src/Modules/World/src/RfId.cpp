//                                - RFID.CPP -
//
//   Implementation of class "CRfId".
//
//   Author: Zhanglei
//   Date:   2004. 5. 25
//

#include "stdafx.h"
#include <memory.h>
#include "rfid.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
// Implementation of class "CRfId".

CRfId::CRfId() 
{
	memset(m_uchCode, 0, RFID_CODE_LEN);
}

CRfId::CRfId(UCHAR *pBuf) 
{
	Init(pBuf);
}

//
//   Initialize the codes.
//
void CRfId::Init(UCHAR* pBuf)
{
	if (pBuf == NULL)
		memset(m_uchCode, 0, RFID_CODE_LEN);
	else
		memcpy(m_uchCode, pBuf, RFID_CODE_LEN);
}

//
//   Compare if 2 RF-IDs are equal.
//
bool CRfId::operator == (CRfId& Obj)	
{
	return (memcmp(m_uchCode, Obj.m_uchCode, RFID_CODE_LEN) == 0);
}

//
//   Compare if 2 RF-IDs are not equal.
//
bool CRfId::operator != (CRfId& Obj) 
{
	return (memcmp(m_uchCode, Obj.m_uchCode, RFID_CODE_LEN) != 0);
}

//
//   Check if the RF-ID is usable.
//
bool CRfId::Usable()
{
	for (int i = 0; i < RFID_CODE_LEN; i++)
		if (m_uchCode[i] != 0)
			return true;

	return false;
}

//
//    Create the node data from a text file.
//
bool CRfId::Create(FILE *StreamIn)
{
   for (int i = 0; i < RFID_CODE_LEN; i++)
   {
      if (fscanf(StreamIn, ", %X", &m_uchCode[i]) == EOF)
         return false;
   }
   fscanf(StreamIn, "\n");
   return true;
}

//
//    Save: Save node data to a text file.
//
bool CRfId::Save(FILE *StreamOut)
{
   for (int i = 0; i < RFID_CODE_LEN; i++)
   {
      if (fprintf(StreamOut, ", %02X", m_uchCode[i]) == EOF)
         return false;
   }
   fprintf(StreamOut, "\n");

   return true;
}

//
//   Archive I/O routine.
//
CArchive& operator >> (CArchive& ar, CRfId& Obj)
{
	for (int i = 0; i < RFID_CODE_LEN; i++)
		ar >> Obj.m_uchCode[i];
	return ar;
}

CArchive& operator << (CArchive& ar, CRfId& Obj)
{
	for (int i = 0; i < RFID_CODE_LEN; i++)
		ar << Obj.m_uchCode[i];
	return ar;
}
