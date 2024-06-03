//                          - NVBLACKBOX.CPP -
//
//   Implementation of class "CNVBlackBox".
//
//   Author: Zhanglei , Lv Xiangren
//   Date:   2010. 7. 30
//

#include "stdafx.h"
#include "NvBlackBox.h"
#include "Project.h"
#include "Tools.h"
#include <string.h>
#include <time.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

LONG CNVBlackBox::m_uchOrderCount = 0L;
CCriticalSection CNVBlackBox::m_CritSec;

USHORT (*pNVRamReadUshort)(ULONG) = NULL;
void (*pNVRamWriteUshort)(ULONG,USHORT) = NULL;

UCHAR (*pNVRamReadUchar)(ULONG) = NULL;
void (*pNVRamWriteUchar)(ULONG,UCHAR) = NULL;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CNVBlackBox".

CNVBlackBox::CNVBlackBox()
{
	m_dwBufStartPos = 0;
	m_dwBufEndPos = 0;
	m_dwHeadPos = 0;
	m_dwTailPos = 0;
	m_dwCheckSum = 0;
	
	m_dwBpHeadPos = 0;
	m_dwBpTailPos = 0;
	m_dwBpCheckSum = 0;
	m_RcdByteCount = 0;
	m_pNvRamfile = NULL;

	m_bStoreCheckSum = FALSE;
	m_dwCheckSumStartTime = GetTickCount();	

	m_uPrevData = 0;
}

CNVBlackBox::CNVBlackBox(ULONG dBufStartPos , ULONG dBufEndPos)
{
	//preserved
}

void CNVBlackBox::NVRamSelAddr(UINT uAddr)
{
#if !defined(PLATFORM_WINDOWS_ARMV4I) && !defined(PLATFORM_WINDOWS_SDK1)
    UCHAR uch;
    UINT addr = uAddr;

#if !defined _NVRAM_SIMULATE_
    uch = addr & 0x0000FF;
    _outp(NVRAM_BASE_PORT, uch);

    addr >>= 8;
    uch = addr & 0x0000FF;
    _outp(NVRAM_BASE_PORT+1, uch);

    addr >>= 8;
    uch = addr & 0x0000FF;
    _outp(NVRAM_BASE_PORT+2, uch);
#else
    m_pNvRamfile->Seek( uAddr, CFile::begin );
#endif
#endif
}

void CNVBlackBox::NVRamWrite(UINT uAddr, UCHAR uchData)
{	
#ifdef PLATFORM_WINDOWS_ARMV4I
    if (pNVRamReadUshort == NULL || pNVRamWriteUshort == NULL)
        return;

    union
    {
        USHORT u;
        UCHAR uch[2];
    } DataBuf;

    ULONG addr = uAddr/2;
    USHORT index = uAddr%2;
    DataBuf.u = (*pNVRamReadUshort)(addr);
    m_uPrevData = uchData;

    if (index == 0)
        DataBuf.uch[0] = uchData;
    else if (index == 1)
        DataBuf.uch[1] = uchData;

    (*pNVRamWriteUshort)(addr,DataBuf.u);
#elif defined PLATFORM_WINDOWS_SDK1
    if (pNVRamWriteUchar == NULL)
        return;

    (*pNVRamWriteUchar)(uAddr,uchData);
#else
    NVRamSelAddr(uAddr);

#if !defined _NVRAM_SIMULATE_
    _outp(NVRAM_BASE_PORT+3, uchData);
#else
    m_pNvRamfile->Write( &uchData, 1 );
#endif
#endif
}

UCHAR CNVBlackBox::NVRamRead(UINT uAddr)
{
    UCHAR ch = 0;

#ifdef PLATFORM_WINDOWS_ARMV4I
    if (pNVRamReadUshort == NULL)
        return 0;

    union
    {
        USHORT u;
        UCHAR uch[2];
    } DataBuf;

    ULONG addr = uAddr/2;
    USHORT index = uAddr%2;
    DataBuf.u = (*pNVRamReadUshort)(addr);

    if (index == 0)
        ch = DataBuf.uch[0];
    else if (index == 1)
        ch = DataBuf.uch[1];
#elif defined PLATFORM_WINDOWS_SDK1
    if (pNVRamReadUchar == NULL)
        return 0;

    ch = (*pNVRamReadUchar)(uAddr);
#else
    NVRamSelAddr(uAddr);

#if !defined _NVRAM_SIMULATE_
    ch = _inp(NVRAM_BASE_PORT+3);
#else
    m_pNvRamfile->Read( &ch, 1 );
#endif
#endif
	
	return ch;
}

void CNVBlackBox::NVRamSelAddrOpti(UINT uAddr)
{
#if !defined(PLATFORM_WINDOWS_ARMV4I) && !defined(PLATFORM_WINDOWS_SDK1)
    UCHAR uch;
    UINT addr = uAddr;

#if !defined _NVRAM_SIMULATE_
    uch = addr & 0x0000FF;
    _outp(NVRAM_BASE_PORT, uch);

    if (uch == 0)
    {
        addr >>= 8;
        uch = addr & 0x0000FF;
        _outp(NVRAM_BASE_PORT+1, uch);

        addr >>= 8;
        uch = addr & 0x0000FF;
        _outp(NVRAM_BASE_PORT+2, uch);
    }
#else
    m_pNvRamfile->Seek( uAddr, CFile::begin );
#endif
#endif
}

UCHAR CNVBlackBox::NVRamReadOpti(UINT uAddr)
{
	UCHAR ch = 0;	

#ifdef PLATFORM_WINDOWS_ARMV4I
    if (pNVRamReadUshort == NULL)
        return 0;

    union
    {
        USHORT u;
        UCHAR uch[2];
    } DataBuf;

    ULONG addr = uAddr/2;
    USHORT index = uAddr%2;
    DataBuf.u = (*pNVRamReadUshort)(addr);

    if (index == 0)
        ch = DataBuf.uch[0];
    else if (index == 1)
        ch = DataBuf.uch[1];
#elif defined PLATFORM_WINDOWS_SDK1
    if (pNVRamReadUchar == NULL)
        return 0;

    ch = (*pNVRamReadUchar)(uAddr);
#else
    NVRamSelAddrOpti(uAddr);

#if !defined _NVRAM_SIMULATE_
    ch = _inp(NVRAM_BASE_PORT+3);
#else
    m_pNvRamfile->Read( &ch, 1 );
#endif
#endif
	
	return ch;
}

void CNVBlackBox::NVRamWriteOpti(UINT uAddr, UCHAR uchData)
{	
#ifdef PLATFORM_WINDOWS_ARMV4I
    if (pNVRamWriteUshort == NULL)
        return;

    union
    {
        USHORT u;
        UCHAR uch[2];
    } DataBuf;

    ULONG addr = uAddr/2;
    USHORT index = uAddr%2;

    if (index == 0)
    {
        DataBuf.uch[0] = uchData;
        DataBuf.uch[1] = 0;
        m_uPrevData = uchData;
    }
    else if (index == 1)
    {
        DataBuf.uch[0] = m_uPrevData;
        DataBuf.uch[1] = uchData;
        m_uPrevData = uchData;
    }

    (*pNVRamWriteUshort)(addr,DataBuf.u);
#elif defined PLATFORM_WINDOWS_SDK1
    if (pNVRamWriteUchar == NULL)
        return;

    (*pNVRamWriteUchar)(uAddr,uchData);
#else
    NVRamSelAddrOpti(uAddr);

#if !defined _NVRAM_SIMULATE_
    _outp(NVRAM_BASE_PORT+3, uchData);
#else
    m_pNvRamfile->Write( &uchData, 1 );
#endif
#endif
}

void CNVBlackBox::NVRamWrite(UCHAR uchData)
{		
#if !defined(PLATFORM_WINDOWS_ARMV4I) && !defined(PLATFORM_WINDOWS_SDK1)
    _outp(NVRAM_BASE_PORT+3, uchData);
#endif
}

UCHAR CNVBlackBox::NVRamRead()
{
	UCHAR ch = 0;

#if !defined(PLATFORM_WINDOWS_ARMV4I) && !defined(PLATFORM_WINDOWS_SDK1)
    ch = _inp(NVRAM_BASE_PORT+3);
#endif

	return ch;
}

USHORT CNVBlackBox::NVRamReadUshort(UINT uAddr)
{
	USHORT u = 0;

#ifdef PLATFORM_WINDOWS_ARMV4I
	if (pNVRamReadUshort == NULL)
		return 0;

    ULONG addr = uAddr/2;
	u = (*pNVRamReadUshort)(addr);

#elif defined PLATFORM_WINDOWS_SDK1
	if (pNVRamReadUchar == NULL)
		return 0;

	UCHAR *p = (UCHAR*)&u;
	*p++ = (*pNVRamReadUchar)(uAddr++);
	*p = (*pNVRamReadUchar)(uAddr);
#else
	UCHAR *p = (UCHAR*)&u;
	*p++ = NVRamRead(uAddr++);
	*p = NVRamReadOpti(uAddr);
#endif

	return u;
}

void CNVBlackBox::NVRamWriteUshort(UINT uAddr, USHORT u)
{	
#ifdef PLATFORM_WINDOWS_ARMV4I
	if (pNVRamWriteUshort == NULL)
		return;

    ULONG addr = uAddr/2;
	(*pNVRamWriteUshort)(addr,u);
#elif defined PLATFORM_WINDOWS_SDK1
	if (pNVRamWriteUchar == NULL)
		return;

	UCHAR *p = (UCHAR*)&u;
	(*pNVRamWriteUchar)(uAddr++, *p++);
	(*pNVRamWriteUchar)(uAddr,*p);
#else
	UCHAR *p = (UCHAR*)&u;
	NVRamWrite(uAddr++, *p++);
	NVRamWriteOpti(uAddr,*p);
#endif
}

ULONG CNVBlackBox::NVRamReadUlong(UINT uAddr)
{
	ULONG u = 0;

#ifdef PLATFORM_WINDOWS_ARMV4I
	if (pNVRamReadUshort == NULL)
		return 0;

	USHORT *p = (USHORT*)&u;
    ULONG addr = uAddr/2;

	*p++ = (*pNVRamReadUshort)(addr++);
	*p = (*pNVRamReadUshort)(addr);

#elif defined PLATFORM_WINDOWS_SDK1
	if (pNVRamReadUchar == NULL)
		return 0;

	UCHAR *p = (UCHAR*)&u;
	for (int i = 0; i < 3; i++)
	{
		*p++ = (*pNVRamReadUchar)(uAddr++);
	}
	*p = (*pNVRamReadUchar)(uAddr);
#else
	UCHAR *p = (UCHAR*)&u;

	*p++ = NVRamRead(uAddr++);
	for (int i = 1; i < 4; i++)
	{
		*p++ = NVRamReadOpti(uAddr++);
	}
#endif

	return u;
}

void CNVBlackBox::NVRamWriteUlong(UINT uAddr, ULONG u)
{
#ifdef PLATFORM_WINDOWS_ARMV4I
	if (pNVRamWriteUshort == NULL)
		return;

	USHORT *p = (USHORT*)&u;
    ULONG addr = uAddr/2;

	(*pNVRamWriteUshort)(addr++,*p++);
	(*pNVRamWriteUshort)(addr,*p);
#elif defined PLATFORM_WINDOWS_SDK1
	if (pNVRamWriteUchar == NULL)
		return;

	UCHAR *p = (UCHAR*)&u;	

	for (int i = 0; i < 3; i++)
	{
		(*pNVRamWriteUchar)(uAddr++, *p++);
	}
	(*pNVRamWriteUchar)(uAddr,*p);
#else
	UCHAR *p = (UCHAR*)&u;	

	NVRamWrite(uAddr++, *p++);
	for (int i = 1; i < 4; i++)
	{
		NVRamWriteOpti(uAddr++, *p++);
	}
#endif
}


//
//   生成黑匣子。
//   return value    0 -- create NV BlackBox failure
//                   1 -- create NV BlackBox success 
//                   2 -- NV BlackBox already exist
//
int CNVBlackBox::Create(char* strFileName , ULONG dBufStartPos , ULONG dBufEndPos )
{
	UCHAR pFileName[10];
	UCHAR pBpFileName[10];
	int retValue = 0;

	BOOL Error = FALSE;
	BOOL BpError = FALSE;

	m_dwBufStartPos = dBufStartPos;
	m_dwBufEndPos = dBufEndPos;
	m_dwHeadPos = 0;
	m_dwTailPos = 0;
	m_dwCheckSum = 0;

	m_dwBpHeadPos = 0;
	m_dwBpTailPos = 0;
	m_dwBpCheckSum = 0;
	m_pNVRamName = strFileName;

	ASSERT(m_dwBufStartPos < m_dwBufEndPos);
	ASSERT(m_dwBufStartPos >= 0 && m_dwBufStartPos <= NVRAM_BUF_MAXPOS);
	ASSERT(m_dwBufEndPos >= 0 && m_dwBufEndPos <= NVRAM_BUF_MAXPOS);

	if (m_dwBufStartPos == m_dwBufEndPos || m_dwBufStartPos > m_dwBufEndPos)
		return 0;
	if (m_dwBufStartPos < 0 || m_dwBufStartPos > NVRAM_BUF_MAXPOS || m_dwBufEndPos < 0 || m_dwBufEndPos > NVRAM_BUF_MAXPOS)
		return 0;

	ObtainBBName(pFileName);
	ObtainBpBBName(pBpFileName);

	//just used to clear the nv ram section at the beginning  
#if defined CLEAR_NVRAM_CONTENT
	Clear();
#endif

	// 核对黑匣子名称及校验和
	// 如果名字不符，则视为原来不存在黑匣子，并立即新建一个
	if ((strcmp(strFileName , (char*)pFileName) != 0) && (strcmp(strFileName , (char*)pBpFileName) != 0))
	{					
		Clear();
		CreateBBName(strFileName);
		CreateBpBBName(strFileName);
		retValue = 1;		
	}
	else
	{					
		// 取得检验和
		m_dwCheckSum = NVRamReadUlong(m_dwBufStartPos + 10);
		m_dwHeadPos = NVRamReadUlong(m_dwBufStartPos + 14);
		m_dwTailPos = NVRamReadUlong(m_dwBufStartPos + 18);

		m_dwBpCheckSum = NVRamReadUlong(m_dwBufStartPos + 30);
		m_dwBpHeadPos = NVRamReadUlong(m_dwBufStartPos + 22);
		m_dwBpTailPos = NVRamReadUlong(m_dwBufStartPos + 26);

		if(m_dwHeadPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwHeadPos > m_dwBufEndPos || m_dwTailPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwTailPos > m_dwBufEndPos)
			Error = TRUE;

		if(m_dwBpHeadPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwBpHeadPos > m_dwBufEndPos || m_dwBpTailPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwBpTailPos > m_dwBufEndPos)
			BpError = TRUE;

		// 如检验和不符，认为该黑匣子已崩溃
		if (Error || m_dwCheckSum != CalCheckSum(m_dwBufStartPos + 14)) 
		{				
			if (BpError || m_dwBpCheckSum != CalCheckSum(m_dwBufStartPos + 22)) 
			{	
				Clear();
				retValue = 1;
			}
			else
			{	
				m_dwCheckSum = m_dwBpCheckSum;    
				m_dwHeadPos = m_dwBpHeadPos;     
				m_dwTailPos = m_dwBpTailPos; 
				NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
				NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);	
				NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
				retValue = 2;
			}
		}
		// 如检验和相符，认为该黑匣子原已存在并可用
		else
		{	
			m_dwBpCheckSum = m_dwCheckSum;    
			m_dwBpHeadPos = m_dwHeadPos;     
			m_dwBpTailPos = m_dwTailPos;  
			NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
			NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
			NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);
			retValue = 2;			
		}
	}

	return retValue;
}

//
//use to simulate the nv ram 
//
int CNVBlackBox::Create(char* strFileName , ULONG dBufStartPos , ULONG dBufEndPos , CFile *pFile)
{
	UCHAR pFileName[10];
	UCHAR pBpFileName[10];
	int retValue = 0;

	BOOL Error = FALSE;
	BOOL BpError = FALSE;

	m_dwBufStartPos = dBufStartPos;
	m_dwBufEndPos = dBufEndPos;
	m_dwHeadPos = 0;
	m_dwTailPos = 0;
	m_dwCheckSum = 0;

	m_dwBpHeadPos = 0;
	m_dwBpTailPos = 0;
	m_dwBpCheckSum = 0;
	m_pNVRamName = strFileName;
	m_pNvRamfile = pFile;

	ASSERT(m_dwBufStartPos < m_dwBufEndPos);
	ASSERT(m_dwBufStartPos >= 0 && m_dwBufStartPos <= NVRAM_BUF_MAXPOS);
	ASSERT(m_dwBufEndPos >= 0 && m_dwBufEndPos <= NVRAM_BUF_MAXPOS);

	if (m_dwBufStartPos == m_dwBufEndPos || m_dwBufStartPos > m_dwBufEndPos)
		return 0;
	if (m_dwBufStartPos < 0 || m_dwBufStartPos > NVRAM_BUF_MAXPOS || m_dwBufEndPos < 0 || m_dwBufEndPos > NVRAM_BUF_MAXPOS)
		return 0;

	ObtainBBName(pFileName);
	ObtainBpBBName(pBpFileName);

	// 核对黑匣子名称及校验和
	// 如果名字不符，则视为原来不存在黑匣子，并立即新建一个
	if ((strcmp(strFileName , (char*)pFileName) != 0) && (strcmp(strFileName , (char*)pBpFileName) != 0))
	{		
		Clear();
		CreateBBName(strFileName);
		CreateBpBBName(strFileName);
		retValue = 1;		
	}
	else
	{		
		// 取得检验和
		m_dwCheckSum = NVRamReadUlong(m_dwBufStartPos + 10);
		m_dwHeadPos = NVRamReadUlong(m_dwBufStartPos + 14);
		m_dwTailPos = NVRamReadUlong(m_dwBufStartPos + 18);

		m_dwBpCheckSum = NVRamReadUlong(m_dwBufStartPos + 30);
		m_dwBpHeadPos = NVRamReadUlong(m_dwBufStartPos + 22);
		m_dwBpTailPos = NVRamReadUlong(m_dwBufStartPos + 26);

		if(m_dwHeadPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwHeadPos > m_dwBufEndPos || m_dwTailPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwTailPos > m_dwBufEndPos)
			Error = TRUE;

		if(m_dwBpHeadPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwBpHeadPos > m_dwBufEndPos || m_dwBpTailPos < m_dwBufStartPos + NVRAM_DATA_OFFSET || m_dwBpTailPos > m_dwBufEndPos)
			BpError = TRUE;

		// 如检验和不符，认为该黑匣子已崩溃
		if (Error || m_dwCheckSum != CalCheckSum(m_dwBufStartPos + 14)) 
		{
			if (BpError || m_dwBpCheckSum != CalCheckSum(m_dwBufStartPos + 22)) 
			{	
				Clear();
				retValue = 1;
			}
			else
			{	
				m_dwCheckSum = m_dwBpCheckSum;    
				m_dwHeadPos = m_dwBpHeadPos;     
				m_dwTailPos = m_dwBpTailPos; 
				NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
				NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);	
				NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
				retValue = 2;
			}
		}
		// 如检验和相符，认为该黑匣子原已存在并可用
		else
		{			
			m_dwBpCheckSum = m_dwCheckSum;    
			m_dwBpHeadPos = m_dwHeadPos;     
			m_dwBpTailPos = m_dwTailPos;  
			NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
			NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
			NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);
			retValue = 2;			
		}
	}

	return retValue;
}

//
//   清除黑匣子内所有内容。
//
void CNVBlackBox::Clear()
{
	m_dwHeadPos = m_dwBufStartPos + NVRAM_DATA_OFFSET;
	m_dwTailPos = m_dwHeadPos;
	m_dwBpHeadPos = m_dwHeadPos;
    m_dwBpTailPos = m_dwHeadPos;
	
	NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);
	NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
	
    m_dwCheckSum = 0;
	NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
	m_dwBpCheckSum = 0;
	NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);

	// 将数据区内容全清为0
    ULONG dBufTempPos = m_dwHeadPos;
	NVRamWrite(dBufTempPos++,0);
    for (ULONG i = m_dwHeadPos + 1; i <= m_dwBufEndPos; i++)
	{
		NVRamWriteOpti(dBufTempPos++,0);
	}	
}

//
//  重置黑匣子
//
void CNVBlackBox::Reset()
{
	m_dwHeadPos = m_dwBufStartPos + NVRAM_DATA_OFFSET;
	m_dwTailPos = m_dwHeadPos;
	m_dwBpHeadPos = m_dwHeadPos;
	m_dwBpTailPos = m_dwHeadPos;

	NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);
	NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);

	m_dwCheckSum = 0;
	NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
	m_dwBpCheckSum = 0;
	NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);
}

//
//   Save the data to a file.    
//
void CNVBlackBox::Save(LPCTSTR strFile)
{
	//preserved
}

CNVBlackBox& CNVBlackBox::operator << (BYTE ch)
{
	NVRamWrite(m_dwTailPos,ch);	
	// 向后移动“队尾”位置
	m_dwTailPos = NextEntry(m_dwTailPos);
	m_RcdByteCount++;
	
	// 如果“队尾”碰到了“队首”,将“队首”也后移
	//当队首移动了，需要移动整条记录，避免乱码	
	if (m_dwTailPos == m_dwHeadPos)
	{	
		for (int i = 0; i < NVRAM_SHIFT_RECORD_AMOUNT; i++)
		{
			BOOL bQuit = TRUE;
			USHORT ucount = 0;

			UCHAR *p = (UCHAR*)&ucount;
			*p++ = NVRamRead(m_dwHeadPos);    //obtain the byte count of per record
			m_dwHeadPos = NextEntry(m_dwHeadPos);
			if (m_dwHeadPos == m_dwBufStartPos + NVRAM_DATA_OFFSET )
				*p = NVRamRead(m_dwHeadPos);   
			else 
				*p = NVRamReadOpti(m_dwHeadPos);  

			if(ucount > 0)
				ucount--;

			if(ucount < 1000  && ucount > 10)
			{
				do {
					m_dwHeadPos = NextEntry(m_dwHeadPos);
					ucount--;

					if (ucount == 2)
					{
						BYTE uch2;
						BYTE uch1 = NVRamRead(m_dwHeadPos);
                        ULONG dPos = NextEntry(m_dwHeadPos);
						if (dPos == m_dwBufStartPos + NVRAM_DATA_OFFSET )
							uch2 = NVRamRead(dPos); 
						else
							uch2 = NVRamReadOpti(dPos); 

						if ((uch1 == 0xAA) && (uch2 == 0xAA))
						{
							m_dwHeadPos = NextEntry(m_dwHeadPos);
							if (m_dwTailPos == m_dwHeadPos)
								break;

							m_dwHeadPos = NextEntry(m_dwHeadPos);
							break;
						}
						else
						{
							m_dwHeadPos = NextEntry(m_dwHeadPos);
							bQuit = FALSE;
							break;
						}
					}

				} while (m_dwTailPos != m_dwHeadPos);
			}
			else
				bQuit = FALSE;

			if(!bQuit)
			{
				BYTE uch1,uch2;
				m_dwHeadPos = NextEntry(m_dwHeadPos);
				uch1 = NVRamRead(m_dwHeadPos); 
				uch2 = uch1;
				int num = 0;

				do {
					uch1 = uch2;
					num++;
                    ULONG dPos = NextEntry(m_dwHeadPos);
					if (dPos == m_dwBufStartPos + NVRAM_DATA_OFFSET )
						uch2 = NVRamRead(dPos); 
					else
						uch2 = NVRamReadOpti(dPos); 

					if ((uch1 == 0xAA) && (uch2 == 0xAA))
					{
						m_dwHeadPos = NextEntry(m_dwHeadPos);
						if (m_dwTailPos == m_dwHeadPos)
							break;

						m_dwHeadPos = NextEntry(m_dwHeadPos);
						break;
					}

					if(num > 500)
					{
						m_dwHeadPos = NextEntry(m_dwHeadPos);
						if (m_dwTailPos == m_dwHeadPos)
							break;

						m_dwHeadPos = NextEntry(m_dwHeadPos);
						break;
					}

					m_dwHeadPos = NextEntry(m_dwHeadPos);

				} while (m_dwTailPos != m_dwHeadPos);
			}
		}

		m_dwBpHeadPos = m_dwHeadPos;		
		m_dwBpCheckSum = CalCheckSumExt(m_dwBpHeadPos,m_dwBpTailPos);   // 重置校验和

		NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
		NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
		NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);

		NVRamWriteUlong(m_dwBufStartPos + 14, m_dwBpHeadPos);
		NVRamWriteUlong(m_dwBufStartPos + 18, m_dwBpTailPos);	
		NVRamWriteUlong(m_dwBufStartPos + 10, m_dwBpCheckSum);
	}

	return *this;
}

CNVBlackBox& CNVBlackBox::operator << (char ch)
{
	*this << (BYTE)ch;

	return *this;
}

CNVBlackBox& CNVBlackBox::operator << (char* p)
{
	BYTE* t = (BYTE*)p;
	while (*t)
		*this << *t++;
		
	return *this;
}

CNVBlackBox& CNVBlackBox::operator << (USHORT u)
{
	BYTE* t = (BYTE*)&u;
	*this << *t++;
	*this << *t;
		
	return *this;
}

CNVBlackBox& CNVBlackBox::operator << (int n)
{
	BYTE* t = (BYTE*)&n;
	for (int i = 0; i < 4; i++)
		*this << *t++;
		
	return *this;
}

//CNVBlackBox& CNVBlackBox::operator << (LONG l)
//{
//    BYTE* t = (BYTE*)&l;
//    for (int i = 0; i < 4; i++)
//        *this << *t++;
		
//    return *this;
//}

CNVBlackBox& CNVBlackBox::operator << (float f)
{	
	BYTE* t = (BYTE*)&f;
	for (int i = 0; i < 4; i++)
		*this << *t++;
		
	return *this;
}

CNVBlackBox& CNVBlackBox::operator << (struct tm *t)
{
//    time_t tm = (ULONG)t.GetTime();    // 取得64位时间表示
    time_t tm = mktime(t);
    BYTE* temp = (BYTE*)&tm;
    for (int i = 0; i < 4; i++)
        *this << *temp++;  // 以二进制方式输出
		
    return *this;
}

void CNVBlackBox::RewriteCriticalData()
{	
	m_dwBpCheckSum = CalCheckSumExt(m_dwBpHeadPos,m_dwBpTailPos);   // 重置校验和

	NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
	NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);

	NVRamWriteUlong(m_dwBufStartPos + 14, m_dwBpHeadPos);
	NVRamWriteUlong(m_dwBufStartPos + 18, m_dwBpTailPos);	
	NVRamWriteUlong(m_dwBufStartPos + 10, m_dwBpCheckSum);
}

void CNVBlackBox::NewRecord()
{
	m_CritSec.Lock();

	//control the nv ram access times
	if ( strcmp(m_pNVRamName , "Nav") == 0 || strcmp(m_pNVRamName , "Laser") == 0 || strcmp(m_pNVRamName , "Custom") == 0)
	{
		if ( GetTickCount() - m_dwCheckSumStartTime > STORE_CHECKSUM_INTERVAL )
		{
			m_bStoreCheckSum = TRUE;
			m_dwCheckSumStartTime = GetTickCount();
		}
		else
		{
			m_bStoreCheckSum = FALSE;
		}
	}	

	m_RcdByteCount = 0;
	//备份队头，队尾和校验和
	m_dwBpCheckSum = m_dwCheckSum;    // 取得当前校验和
	m_dwBpHeadPos = m_dwHeadPos;      // 取得当前“队头”位置
	m_dwBpTailPos = m_dwTailPos;      // 取得当前“队尾”位置

	// 将备份“队头”,“队尾”,校验和 写为缓冲区头位置
	if ( strcmp(m_pNVRamName , "Nav") == 0 || strcmp(m_pNVRamName , "Laser") == 0 || strcmp(m_pNVRamName , "Custom") == 0)
	{
		if (m_bStoreCheckSum)
		{
			NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
			NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
			NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);
		}
	}
	else
	{
		NVRamWriteUlong(m_dwBufStartPos + 22, m_dwBpHeadPos);
		NVRamWriteUlong(m_dwBufStartPos + 26, m_dwBpTailPos);
		NVRamWriteUlong(m_dwBufStartPos + 30, m_dwBpCheckSum);
	}
}

void CNVBlackBox::EndRecord()
{	
	m_dwCheckSum = CalCheckSumExt(m_dwHeadPos,m_dwTailPos);   // 重置校验和

	if ( strcmp(m_pNVRamName , "Nav") == 0 || strcmp(m_pNVRamName , "Laser") == 0 || strcmp(m_pNVRamName , "Custom") == 0)
	{
		if (m_bStoreCheckSum)
		{
			NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
			NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);	
			NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
		}
	}
	else
	{
		NVRamWriteUlong(m_dwBufStartPos + 14, m_dwHeadPos);
		NVRamWriteUlong(m_dwBufStartPos + 18, m_dwTailPos);	
		NVRamWriteUlong(m_dwBufStartPos + 10, m_dwCheckSum);
	}

	m_CritSec.Unlock();	
}
//
//    计算头指针、尾指针校验和。
//
ULONG CNVBlackBox::CalCheckSum(ULONG uPos)
{
	ULONG uSum = 0;

    ULONG dBufStartPos = uPos;
	uSum += (ULONG)NVRamRead(dBufStartPos++);
    for (ULONG i = 1; i < 8; i++)
	{
		uSum += (ULONG)NVRamReadOpti(dBufStartPos++);
	}	

	return uSum;
}

//
//    运行过程中计算头指针、尾指针校验和。
//
ULONG CNVBlackBox::CalCheckSumExt(ULONG uHeadPos,ULONG uTailPos)
{
	ULONG uSum = 0;
	UCHAR *p = (UCHAR*)&uHeadPos;
	UCHAR *q = (UCHAR*)&uTailPos;

	for (int i = 0; i < 4; i++)
	{
		uSum += *p++;
		uSum += *q++;
	}			

	return uSum;
}

//
//     obtain the black box name
//
void CNVBlackBox::ObtainBBName(UCHAR* p)
{
    ULONG dBufStartPos = m_dwBufStartPos;
	UCHAR *t = p;
	
	for (int i = 0; i < 10; i++)
	{
		*t = NVRamRead(dBufStartPos++);
		if(*t == '\0')
			break;
		t++;		
	}	
}

//
//        Create the black box name
//
void CNVBlackBox::CreateBBName(char* strFileName)
{
    ULONG dBufStartPos = m_dwBufStartPos;
	UCHAR* p =(UCHAR*)strFileName;
		
	for (int i = 0; i < 10; i++)
	{
        if (*p == '\0')
		{
			NVRamWrite(dBufStartPos,*p);
			break;
		}	
		NVRamWrite(dBufStartPos++,*p++);		
	}	
}

//
//     obtain the backup black box name
//
void CNVBlackBox::ObtainBpBBName(UCHAR* p)
{
    ULONG dBufStartPos = m_dwBufStartPos + 34;
	UCHAR *t = p;

	for (int i = 0; i < 10; i++)
	{
		*t = NVRamRead(dBufStartPos++);
		if(*t == '\0')
			break;
		t++;		
	}	
}

//
//        Create the backup black box name
//
void CNVBlackBox::CreateBpBBName(char* strFileName)
{
    ULONG dBufStartPos = m_dwBufStartPos + 34;
	UCHAR* p =(UCHAR*)strFileName;

	for (int i = 0; i < 10; i++)
	{
		if (*p == '\0')
		{
			NVRamWrite(dBufStartPos,*p);
			break;
		}	
		NVRamWrite(dBufStartPos++,*p++);		
	}	
}

void CNVBlackBox::SetNVRamReadUshort(USHORT (*Proc)(ULONG))
{
	pNVRamReadUshort = Proc;
}

void CNVBlackBox::SetNVRamWriteUshort(void (*Proc)(ULONG,USHORT))
{
	pNVRamWriteUshort = Proc;
}

void CNVBlackBox::SetNVRamReadUchar(UCHAR (*Proc)(ULONG))
{
	pNVRamReadUchar = Proc;
}

void CNVBlackBox::SetNVRamWriteUchar(void (*Proc)(ULONG,UCHAR))
{
	pNVRamWriteUchar = Proc;
}
