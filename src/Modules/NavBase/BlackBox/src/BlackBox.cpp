//                          - BLACKBOX.CPP -
//
//   Implementation of class "CBlackBox".
//
//   Author: Zhanglei
//   Date:   2003. 2. 18
//
#include <string.h>
#include "BlackBox.h"
#include "Project.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

ULONG CBlackBox::m_ulTotalCount = 0L;
ULONG* CBlackBox::m_pReserveData[] = {NULL};
std::mutex CBlackBox::reserve_mtx;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CBlackBox".

CBlackBox::CBlackBox()
{
   m_pHead = m_pTail = NULL;
   m_pCheck = NULL;
   m_pMem = NULL;
   m_hSynEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
   m_RcdByteCount = 0;
   m_pSaveHead = NULL;
   m_pSaveTail = NULL;
   m_bFirstTime = TRUE;
   m_bCreateBlackboxOK = FALSE;
   m_bUseNvramBlackbox = FALSE;
   m_strBoxFileName ="";

}

//
//   Create the black box.
//
BOOL CBlackBox::Create(CString strFileName, ULONG dwMaxLen)
{

   //打开内存映射文件
   int nOpenStatus = OpenFileMap(strFileName, dwMaxLen);
   if (nOpenStatus < 0)
      return FALSE;

   // Init the pointer to the head/tail/checker
   m_pHead = 0;
   m_pTail = m_pHead;
   m_pCheck = m_pTail+1;

   m_pSaveHead = m_pCheck+1;
   m_pSaveTail = m_pSaveHead+1;

   m_pMem = (char*)m_pBuffer;

   m_nMaxCount = dwMaxLen-20;

   if (nOpenStatus == 1)           // File created successfully
   {
      Clear();
   }
   else if (nOpenStatus == 2)     // File already exists
   {
      // Verify the checker here ..
      // ...
   }
   m_bCreateBlackboxOK = TRUE;


   return TRUE;
}
BOOL CBlackBox::Create( CString strFileName, void *dBufStartBasePos , ULONG dStartOffset , ULONG dwMaxLen )
{
    CString strPathName = (LOG_FILE_PATH);
    strPathName += strFileName;
    m_strBoxFileName = strPathName;

    m_MaxSize = dwMaxLen;
    // Init the pointer to the head/tail/checker
    m_pHead = (ULONG*)((char*)dBufStartBasePos + dStartOffset);
    m_pTail =  m_pHead + 1;
    m_pCheck = m_pTail + 1;

    m_pSaveHead = m_pCheck+1;
    m_pSaveTail = m_pSaveHead+1;

    m_pMem  =(char*)(m_pSaveTail+1);

    m_nMaxCount = dwMaxLen - 20;   // five ULONG need 20 char

    if (m_pMem != NULL)            // File created successfully
    {
       Clear();
    }

    m_bCreateBlackboxOK = TRUE;

    return  TRUE;
}
//
//   Create the black box.
//
BOOL CBlackBox::Create(CString strFileName, ULONG dwMaxLen , char* nvRamBBName ,
                       ULONG dBufStartPos , ULONG dBufEndPos, void *dBufStartBasePos, ULONG dStartOffset)
{
    int retValue =0;
/********************** share mem ************************/
    CString strPathName = (LOG_FILE_PATH);
    strPathName += strFileName;
    m_strBoxFileName = strPathName;

    m_MaxSize = dwMaxLen;
    // Init the pointer to the head/tail/checker
    m_pHead = (ULONG*)((char*)dBufStartBasePos + dStartOffset);
    m_pTail =  m_pHead + 1;
    m_pCheck = m_pTail + 1;
    m_pSaveHead = m_pCheck+1;
    m_pSaveTail = m_pSaveHead+1;

    m_pMem  =(char*)(m_pSaveTail+1);

    m_nMaxCount = dwMaxLen - 20;   // five ULONG need 20 char
/************************************************************/

//    if (!m_hSynEvent)
//    {
//        return FALSE;
//    }

#ifdef USE_NVRAM_BLACKBOX
//  if (WaitForSingleObject(m_hSynEvent,INFINITE) == WAIT_OBJECT_0)
    {
        retValue = m_NvRamBox.Create(nvRamBBName , dBufStartPos , dBufEndPos );

        ULONG& nTail = *m_pTail;
        ULONG& nHead = *m_pHead;
//        if (nOpenStatus == 1)           // File created successfully
//        {
//            Clear();
//        }
//        else if (nOpenStatus == 2)     // File already exists
//        {
//            // Verify the checker here ..
//            // ...
//        }
        if (m_pMem != NULL)            // File created successfully
        {
           Clear();
        }
        else
        {
            return FALSE;
        }


        if (m_bFirstTime)
            Clear();  //firstly clear the black box content

        if (retValue == 0)
        {
            // Create NVBlackBox failure
        }
        else if(retValue == 1)
        {
            Clear();
        }
        else if (retValue == 2)  // Load the NVblackbox contents from the NVRam
        {
            ULONG dBufStart = m_NvRamBox.m_dwBufStartPos + NVRAM_DATA_OFFSET;
            m_pMem[nTail] = (BYTE)m_NvRamBox.NVRamRead(dBufStart++);

            for (ULONG i = m_NvRamBox.m_dwBufStartPos + NVRAM_DATA_OFFSET + 1; i <= m_NvRamBox.m_dwBufEndPos; i++)
            {
                nTail = NextEntry(nTail);
                m_pMem[nTail] = (BYTE)m_NvRamBox.NVRamReadOpti(dBufStart++);
            }

            nHead = m_NvRamBox.TransformMem(m_NvRamBox.m_dwHeadPos);
            nTail = m_NvRamBox.TransformMem(m_NvRamBox.m_dwTailPos);
        }

        *m_pSaveHead = *m_pHead;
        *m_pSaveTail = *m_pTail;
    }
#endif

//    SetEvent(m_hSynEvent);
    m_bCreateBlackboxOK = TRUE;

    m_bUseNvramBlackbox = TRUE;

    return TRUE;
}

//
//use to simulate the nv ram throught the file
//
BOOL CBlackBox::Create(CString strFileName, ULONG dwMaxLen , char* nvRamBBName , ULONG dBufStartPos , ULONG dBufEndPos ,CFile *pFile)
{
//	int retValue =0;
//	//打开内存映射文件
//	int nOpenStatus = OpenFileMap(strFileName, dwMaxLen);
//	if (nOpenStatus < 0)
//		return FALSE;

//	// Init the pointer to the head/tail/checker
//	m_pHead = (ULONG*)m_pBuffer;
//	m_pTail = m_pHead+1;
//	m_pCheck = m_pTail+1;

//	m_pSaveHead = m_pCheck+1;
//	m_pSaveTail = m_pSaveHead+1;

//	m_pMem = (char*)(m_pSaveTail+1);
//	m_nMaxCount = dwMaxLen-20;

//	if (!m_hSynEvent)
//	{
//		return FALSE;
//	}

//	if (WaitForSingleObject(m_hSynEvent,INFINITE) == WAIT_OBJECT_0)
//	{
//		retValue = m_NvRamBox.Create(nvRamBBName , dBufStartPos , dBufEndPos ,pFile);

//		ULONG& nTail = *m_pTail;
//		ULONG& nHead = *m_pHead;
//		if (nOpenStatus == 1)           // File created successfully
//		{
//			Clear();
//		}
//		else if (nOpenStatus == 2)     // File already exists
//		{
//			// Verify the checker here ..
//			// ...
//		}

//		if (retValue == 0)
//		{
//			// Create NVBlackBox failure
//		}
//		else if(retValue == 1)
//		{
//			Clear();
//		}
//		else if (retValue == 2)  // Load the NVblackbox contents from the NVRam
//		{
//			ULONG dBufStart = m_NvRamBox.m_dwBufStartPos + NVRAM_DATA_OFFSET;
//			m_pMem[nTail] = (BYTE)m_NvRamBox.NVRamRead(dBufStart++);

//			for (ULONG i = m_NvRamBox.m_dwBufStartPos + NVRAM_DATA_OFFSET + 1; i <= m_NvRamBox.m_dwBufEndPos; i++)
//			{
//				nTail = NextEntry(nTail);
//				m_pMem[nTail] = (BYTE)m_NvRamBox.NVRamReadOpti(dBufStart++);
//			}

//			nHead = m_NvRamBox.TransformMem(m_NvRamBox.m_dwHeadPos);
//			nTail = m_NvRamBox.TransformMem(m_NvRamBox.m_dwTailPos);
//		}

//		*m_pSaveHead = *m_pHead;
//		*m_pSaveTail = *m_pTail;
//	}

//	SetEvent(m_hSynEvent);
//	m_bCreateBlackboxOK = TRUE;

    return TRUE;
}

//
//   Clear the black box
//
void CBlackBox::Clear()
{
    *m_pHead = 0;
    *m_pTail = 0;

    if (m_bFirstTime)
        m_bFirstTime = FALSE;

    //m_pSaveHead = NULL;
    //m_pSaveTail = NULL;

    *m_pSaveHead = 0;
    *m_pSaveTail = 0;


   // Cheker operation ..
   // ..
}


void CBlackBox::Save(LPCTSTR strFile)
{
    if(m_bUseNvramBlackbox)
    {
        FILE *file ;

        CString strFileName = strFile;
        strFileName += _T("Box.bin");

        //打开文件
        if((file = fopen(strFileName.c_str() , "wb")) == NULL)
        {
            perror("open") ;
        }

        ULONG &nTail = *m_pSaveTail;
        ULONG &nHead = *m_pSaveHead;

        // If the buffer is not rewinded, just write from start of buffer to the tail
        if (nTail > nHead)
        {
            fwrite(m_pMem,1,nTail-nHead,file);
        }
        // If the buffer is rewinded, do the writing in 2 stages
        else if (nTail != nHead)
        {
            fwrite(&m_pMem[nHead],1, m_nMaxCount-nHead,file);
            fwrite(m_pMem,1,nTail,file);
        }

        fclose(file);

    }
    else
    {
        FILE *file ;

        CString strFileName = strFile;
        strFileName += _T("Box.txt");

        //打开文件
        if((file = fopen(strFileName.c_str() , "w")) == NULL)
        {
            perror("open") ;
        }

        ULONG &nTail = *m_pTail;
        ULONG &nHead = *m_pHead;

        // If the buffer is not rewinded, just write from start of buffer to the tail
        if (nTail > nHead)
        {
            fwrite(m_pMem,1,nTail-nHead,file);
        }
        // If the buffer is rewinded, do the writing in 2 stages
        else if (nTail != nHead)
        {
            fwrite(&m_pMem[nHead],1, m_nMaxCount-nHead,file);
            fwrite(m_pMem,1,nTail,file);
        }

        fclose(file);
    }
}
//
//   Save the data to a file.
//
void CBlackBox::Save(/*LPCTSTR strFile*/)
{
    if(m_bUseNvramBlackbox)
    {
        FILE *file ;

        CString strFileName = m_strBoxFileName;
        strFileName += _T("Box.bin");

        //打开文件
        if((file = fopen(strFileName.c_str() , "wb")) == NULL)
        {
            perror("open") ;
        }

        ULONG &nTail = *m_pSaveTail;
        ULONG &nHead = *m_pSaveHead;

        // If the buffer is not rewinded, just write from start of buffer to the tail
        if (nTail > nHead)
        {
            fwrite(m_pMem,1,nTail-nHead,file);
        }
        // If the buffer is rewinded, do the writing in 2 stages
        else if (nTail != nHead)
        {
            fwrite(&m_pMem[nHead],1, m_nMaxCount-nHead,file);
            fwrite(m_pMem,1,nTail,file);
        }

        fclose(file);

    }
    else
    {
        FILE *file ;

        CString strFileName = m_strBoxFileName;
        strFileName += _T("Box.txt");

        //打开文件
        if((file = fopen(strFileName.c_str() , "w")) == NULL)
        {
            perror("open") ;
        }

        ULONG &nTail = *m_pTail;
        ULONG &nHead = *m_pHead;

        // If the buffer is not rewinded, just write from start of buffer to the tail
        if (nTail > nHead)
        {
            fwrite(m_pMem,1,nTail-nHead,file);
        }
        // If the buffer is rewinded, do the writing in 2 stages
        else if (nTail != nHead)
        {
            fwrite(&m_pMem[nHead],1, m_nMaxCount-nHead,file);
            fwrite(m_pMem,1,nTail,file);
        }

        fclose(file);
    }
}

CBlackBox& CBlackBox::operator << (BYTE ch)
{
    ULONG &nTail = *m_pTail;
    ULONG &nHead = *m_pHead;

//    m_pMem[m_pTail] = ch;

    memcpy(m_pMem+nTail, &ch, 1);


    nTail = NextEntry(nTail);

    if(m_bUseNvramBlackbox)
    {
#ifdef USE_NVRAM_BLACKBOX
        m_RcdByteCount++;
        // If the tail has touched the head, proceed the head to the start of the next record
        if (nTail == nHead)
        {
            for (int i = 0; i < NVRAM_SHIFT_RECORD_AMOUNT; i++)
            {
                BOOL bQuit = TRUE;
                USHORT ucount = 0;            //obtain the byte count of per record
                BYTE *p = (BYTE*)&ucount;
                *p++ = m_pMem[nHead];
                nHead = NextEntry(nHead);
                *p = m_pMem[nHead];

                if(ucount > 0)
                    ucount--;

                if(ucount < 1000  && ucount > 10)
                {
                    do {
                        nHead = NextEntry(nHead);
                        ucount--;

                        if (ucount == 2)
                        {
                            BYTE uch1 = m_pMem[nHead];
                            BYTE uch2 = m_pMem[NextEntry(nHead)];

                            if ((uch1 == 0xAA) && (uch2 == 0xAA))
                            {
                                nHead = NextEntry(nHead);
                                if (nTail == nHead)
                                    break;

                                nHead = NextEntry(nHead);
                                break;
                            }
                            else
                            {
                                nHead = NextEntry(nHead);
                                bQuit = FALSE;
                                break;
                            }
                        }

                    } while (nTail != nHead);
                }
                else
                    bQuit = FALSE;

                if(!bQuit)
                {
                    int num = 0;
                    do {
                        nHead = NextEntry(nHead);
                        BYTE uch1 = m_pMem[nHead];
                        BYTE uch2 = m_pMem[NextEntry(nHead)];
                        num++;

                        if ((uch1 == 0xAA) && (uch2 == 0xAA))
                        {
                            nHead = NextEntry(nHead);
                            if (nTail == nHead)
                                break;

                            nHead = NextEntry(nHead);
                            break;
                        }

                        if (num > 500)
                        {
                            nHead = NextEntry(nHead);
                            if (nTail == nHead)
                                break;

                            nHead = NextEntry(nHead);
                            break;
                        }

                    } while (nTail != nHead);
                }
            }

            m_NvRamBox.m_dwBpHeadPos = m_NvRamBox.TransformRam(nHead);
            m_NvRamBox.RewriteCriticalData();
            *m_pSaveHead = nHead;
        }
#endif
    }
    else
    {
        //sfe1012 delete it
        // If the tail has touched the head, proceed the head to the start of the next line
        if (nTail == nHead)
        {
            do {
                    nHead = NextEntry(nHead);
                    BYTE uch1 = m_pMem[nHead];
//                  BYTE uch2 = m_pMem[NextEntry(nHead)];  //sfe1012 delete it
                    BYTE uch2 = m_pMem[NextEntryNotPlus(nHead)]; //sfe1012 not ++

                    if ((uch1 == '\r') && (uch2 == '\n'))
                    {
                        nHead = NextEntry(nHead);  //sfe1012 add
                        nHead = NextEntry(nHead);
                        if (nTail == nHead)
                            break;

                        nHead = NextEntry(nHead);
                        break;
                    }

            }while (nTail != nHead);
        }
    }
    return *this;
}

CBlackBox& CBlackBox::operator << (char ch)
{
    if(m_bUseNvramBlackbox)
        *this << (BYTE)ch;
    else
        *this << (BYTE)ch;

    return *this;
}

CBlackBox& CBlackBox::operator << (char* p)
{
    if(m_bUseNvramBlackbox)
    {
        BYTE* t = (BYTE*)p;
        while (*t)
            *this << *t++;
    }
    else
    {
        BYTE* t = (BYTE*)p;
        while (*t)
            *this << *t++;
    }

    return *this;
}


CBlackBox& CBlackBox::operator << (USHORT u)
{
    if(m_bUseNvramBlackbox)
    {
        BYTE* t = (BYTE*)&u;
        *this << *t++;
        *this << *t;
    }
    else
    {
        sprintf(m_chText, "%d", u);
        *this << m_chText;
    }

    return *this;
}

CBlackBox& CBlackBox::operator << (int n)
{
    if(m_bUseNvramBlackbox)
    {
        BYTE* t = (BYTE*)&n;
        for (int i = 0; i < 4; i++)
            *this << *t++;
    }
    else
    {
        sprintf(m_chText, "%d", n);
        *this << m_chText;
    }

   return *this;
}

#ifndef _LINUX64
    #ifndef  AGV_LINUX_DEBUG
    CBlackBox& CBlackBox::operator << (LONG l)
    {
        if(m_bUseNvramBlackbox)
        {
            BYTE* t = (BYTE*)&l;
            for (int i = 0; i < 4; i++)
                *this << *t++;
        }
        else
        {
            sprintf(m_chText, "%ld", l);
            *this << m_chText;
        }

       return *this;
    }
    #endif
#endif

CBlackBox& CBlackBox::operator << (float f)
{
    if(m_bUseNvramBlackbox)
    {
        BYTE* t = (BYTE*)&f;
        for (int i = 0; i < 4; i++)
            *this << *t++;
    }
    else
    {
        sprintf(m_chText, "%f", f);
        *this << m_chText;
    }

    return *this;
}

CBlackBox& CBlackBox::operator << (struct tm *t)
{
    struct tm * lpstLocalTime = t;
    if(m_bUseNvramBlackbox)
    {
//        time_t tm = (ULONG)t.GetTime();    // 取得64位时间表示
        time_t tm = mktime(t);
        BYTE* temp = (BYTE*)&tm;
        for (int i = 0; i < 4; i++)
            *this << *temp++;  // 以二进制方式输出
    }
    else
    {
        if(lpstLocalTime)
        {
            sprintf(m_chText, "%04d-%02d-%02d %02d:%02d:%02d",
                    lpstLocalTime->tm_year + 1900,
                    lpstLocalTime->tm_mon + 1,
                    lpstLocalTime->tm_mday,
                    lpstLocalTime->tm_hour,
                    lpstLocalTime->tm_min,
                    lpstLocalTime->tm_sec
                );
            *this << m_chText;
        }
    }

    return *this;
}

void CBlackBox::NewRecord()
{
    ULONG &nTail = *m_pTail;

    m_CritSection.Lock();

    if(m_bUseNvramBlackbox)
    {
#ifdef USE_NVRAM_BLACKBOX
        *m_pSaveHead = *m_pHead;
        *m_pSaveTail = *m_pTail;

        m_NvRamBox.NewRecord();

        m_RcdByteCount = 0;
        m_RcdByteAdr = nTail;

        *this << m_RcdByteCount << (LONG)m_ulTotalCount << GetCurrentTime();
        m_ulTotalCount++;
#endif
    }
    else
    {
        if ((*m_pHead == 0) && (*m_pTail == 0))
        {
            char* pp = VER_IN_BLACKBOX;
            *this << pp << "\r\n";
        }

        *this << "(" << (int)m_ulTotalCount << ",\t"<< GetCurrentTime() << ") ";
        m_ulTotalCount++;
    }
}

void CBlackBox::EndRecord()
{
    if(m_bUseNvramBlackbox)
    {
#ifdef USE_NVRAM_BLACKBOX
        *this << (USHORT)0xAAAA;   //添加校验码

        ULONG tempAdr = m_RcdByteAdr;
        BYTE* t = (BYTE*)&m_RcdByteCount;
        m_pMem[tempAdr] = *t++;
        tempAdr = NextEntry(tempAdr);
        m_pMem[tempAdr] = *t;

        //Input the data to NVRAM
        ULONG nHead = m_RcdByteAdr;
        ULONG nTail = *m_pTail;

        if (nHead < nTail)
        {
            ULONG dBufStart = m_NvRamBox.TransformRam(nHead);
            m_NvRamBox.NVRamWrite(dBufStart,m_pMem[nHead]);

            for (ULONG i = m_RcdByteAdr + 1; i < nTail; i++)
            {
                nHead = NextEntry(nHead);
                dBufStart = m_NvRamBox.TransformRam(nHead);
                m_NvRamBox.NVRamWriteOpti(dBufStart,m_pMem[nHead]);
            }
        }
        else if (nHead > nTail)
        {
            ULONG dBufStart = m_NvRamBox.TransformRam(nHead);
            m_NvRamBox.NVRamWrite(dBufStart,m_pMem[nHead]);

            for (ULONG j = m_RcdByteAdr + 1; j < m_nMaxCount; j++)
            {
                nHead = NextEntry(nHead);
                dBufStart = m_NvRamBox.TransformRam(nHead);
                m_NvRamBox.NVRamWriteOpti(dBufStart,m_pMem[nHead]);
            }

            nHead = 0;
            dBufStart = m_NvRamBox.TransformRam(nHead);
            m_NvRamBox.NVRamWrite(dBufStart,m_pMem[nHead]);

            for (ULONG k = 1; k < nTail; k++)
            {
                nHead = NextEntry(nHead);
                dBufStart = m_NvRamBox.TransformRam(nHead);
                m_NvRamBox.NVRamWriteOpti(dBufStart,m_pMem[nHead]);
            }
        }

        m_NvRamBox.m_dwHeadPos = m_NvRamBox.TransformRam(*m_pHead);
        m_NvRamBox.m_dwTailPos = m_NvRamBox.TransformRam(*m_pTail);

        m_NvRamBox.EndRecord();

        *m_pSaveHead = *m_pHead;
        *m_pSaveTail = *m_pTail;

        m_CritSection.Unlock();
#endif
    }
    else
    {
        *this << "\r\n";
        m_CritSection.Unlock();
    }
}


void CBlackBox::CreateReserveData(void *dBufStartBasePos , ULONG dStartOffset)
{
    m_pReserveData[0] = (ULONG*)((char*)dBufStartBasePos + dStartOffset);
    *m_pReserveData[0] = NO_ACTION_BLACKBOX;
}

