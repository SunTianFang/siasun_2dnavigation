
//                          - BLACKBOX.H -
//
//   The interface of class "CBlackBox".
//
//   Author: Zhanglei
//   Date:   2003. 2. 18
//

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include "ZTypes.h"
#include "MemFileMap.h"
#include "NvBlackBox.h"
#include <mutex>

#define RESERVE_DATA_SIZE   10

enum TBlackBoxAction
{
    NO_ACTION_BLACKBOX = 0,
    EXPORT_BLACKBOX
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CBlackBox".
class DllExport CBlackBox : public CMemFileMap
{
public:
   ULONG*   m_pHead;              // Pointer to the head
   ULONG*   m_pTail;              // Pointer to the tail
   ULONG*   m_pCheck;             // Pointer to the check
   char*    m_pMem;               // Pointer to the memory area
   ULONG    m_nMaxCount;          // Max. data count (in byte)
   char     m_chText[100];
   CCriticalSection m_CritSection;

   HANDLE m_hSynEvent;          // 写NVRAM时，同步事件句柄

#ifdef USE_NVRAM_BLACKBOX
   CNVBlackBox m_NvRamBox;
#endif
   USHORT m_RcdByteCount;       //every record byte count
   ULONG  m_RcdByteAdr;          // 指向用于保存每条记录字节数的地址

   ULONG*   m_pSaveHead;          // Pointer to the head to be saved to the file
   ULONG*   m_pSaveTail;          // Pointer to the tail to be saved to the file
   BOOL    m_bFirstTime;
   BOOL	   m_bCreateBlackboxOK;
   BOOL    m_bUseNvramBlackbox;

   static ULONG m_ulTotalCount;
   static ULONG* m_pReserveData[RESERVE_DATA_SIZE];
   static std::mutex reserve_mtx;

   CString m_strBoxFileName;

private:
   inline ULONG NextEntry(ULONG &dwEntry)
   {
      if (++dwEntry >= m_nMaxCount)
         return 0;
      else
         return dwEntry;
   }

   inline ULONG NextEntryNotPlus(ULONG &dwEntry)
   {
       ULONG dwEntryTemp = dwEntry;
      if (++dwEntryTemp >= m_nMaxCount)
         return 0;
      else
         return dwEntryTemp;
   }


public:
	CBlackBox();

	// Create the black box
    BOOL Create(CString strFileName, ULONG dwMaxLen = 512000);

    //sfe add for share mem black
    BOOL Create(CString strFileName, void *dBufStartBasePos , ULONG dStartOffset , ULONG dwMaxLen = 512000);

    BOOL Create(CString strFileName, ULONG dwMaxLen , char* nvRamBBName ,               
                ULONG dBufStartPos, ULONG dBufEndPos, void *dBufStartBasePos, ULONG dStartOffset);

    BOOL Create(CString strFileName, ULONG dwMaxLen , char* nvRamBBName , ULONG dBufStartPos , ULONG dBufEndPos ,CFile *pFile);
	// Clear the black box
	void Clear();
	// Save the data to a file
    void Save(/*LPCTSTR strFile*/);
    void Save(LPCTSTR strFile);

	CBlackBox& operator << (BYTE ch);
	CBlackBox& operator << (char ch);
	CBlackBox& operator << (char* p);
	CBlackBox& operator << (USHORT u);
	CBlackBox& operator << (int n);
        #ifdef _LINUX64
        #else
            #ifndef  AGV_LINUX_DEBUG
            CBlackBox& operator << (LONG l);
            #endif
        #endif
	CBlackBox& operator << (float f);
    CBlackBox& operator << (struct tm *t);

	void NewRecord();
	void EndRecord();

    static void CreateReserveData(void *dBufStartBasePos , ULONG dStartOffset);

    static void SetReserveData(ULONG lValue)
    {
        {
            std::lock_guard<std::mutex> lock(reserve_mtx);
            if(m_pReserveData[0] != NULL){
                *m_pReserveData[0] = lValue;
            }
        }
    }
    static ULONG GetReserveData()
    {
        ULONG lValue = 0;
        {
            std::lock_guard<std::mutex> lock(reserve_mtx);
            if(m_pReserveData[0] != NULL){
               lValue = *m_pReserveData[0];
            }
        }
        return lValue;
    }

};


#endif /* BLACKBOX_H_ */

