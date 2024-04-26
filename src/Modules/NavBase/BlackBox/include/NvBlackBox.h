//                          - NVBLACKBOX.H -
//
//   The interface of class "CNVBlackBox".
//
//   Author: Zhanglei , Lv Xiangren
//   Date:   2010. 7. 30
//

#ifndef __CNVBlackBox
#define __CNVBlackBox

#include "ZTypes.h"
//#include "Debug.h"

#define NVRAM_DATA_OFFSET             44
#define STORE_CHECKSUM_INTERVAL       500
#define NVRAM_SHIFT_RECORD_AMOUNT     10
#define NVRAM_BUF_MAXPOS              0x0FFFFF


////////////////////////////////////////////////////////////////////////////////////////////
//   非挥发性黑匣子存储格式定义：                                                         //
//                                                                                        //
//   m_dwBufStartPos: 黑匣子缓冲区起始位置(相对于NVRAM空间起始点，下同)                   //
//   m_dwBufEndPos:   黑匣子缓冲区结束位置                                                //
//                                                                                        //
//   其中，存储区头44个字节作为黑匣子的关键数据定义之用，具体如下：                       //
//                                                                                        //
//   字节1~10:   黑匣子名称(10个ASCII字符)                                                //
//   字节11~14:  黑匣子头尾指针校验和(ULONG型)                                            //
//   字节15~18:  当前数据头位置(ULONG型)                                                  //
//   字节19~22:  当前数据尾位置(ULONG型)                                                  //
//   字节23~26:  备份数据头位置(ULONG型)                                                  //
//   字节27~30:  备份数据尾位置(ULONG型)                                                  //
//   字节31~34:  备份黑匣子头尾指针校验和(ULONG型)                                        //
//   字节35~44:  备份黑匣子名称(10个ASCII字符) 
////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CNVBlackBox".
class DllExport CNVBlackBox 
{
public:

    ULONG m_dwBufStartPos;          // 在NVRAM内的内存起始位置
    ULONG m_dwBufEndPos;            // 在NVRAM内的内存结束位置
    ULONG m_dwHeadPos;              // 当前队列头位置
    ULONG m_dwTailPos;              // 当前队列尾位置
    ULONG m_dwCheckSum;             //校验和
    ULONG m_dwBpHeadPos;            // 备份队列头位置
    ULONG m_dwBpTailPos;            // 备份队列尾位置
    ULONG m_dwBpCheckSum;           // 备份校验和

	USHORT m_RcdByteCount;          //every record byte count

	CCriticalSection m_CritSection;
	static CCriticalSection m_CritSec;
	CFile *m_pNvRamfile;

	BOOL  m_bStoreCheckSum;          //whether store the check sum to the nv ram or not
    ULONG m_dwCheckSumStartTime;
	char* m_pNVRamName;

	static LONG m_uchOrderCount;

	UCHAR m_uPrevData;   // 记录上一个字节数据，用于ARM版本

public:
	//计算黑匣子中相对于当前存储位置的下一个有效位置。
    inline ULONG NextEntry(ULONG dwPos)
	{
		if (++dwPos > m_dwBufEndPos)
			return m_dwBufStartPos + NVRAM_DATA_OFFSET;
		else
			return dwPos;
	}

    inline ULONG TransformMem(ULONG dwEntry)
	{
		return (dwEntry - m_dwBufStartPos - NVRAM_DATA_OFFSET);
	}

    inline ULONG TransformRam(ULONG dwEntry)
	{
		return (dwEntry + m_dwBufStartPos + NVRAM_DATA_OFFSET);
	}

public:
	CNVBlackBox();
	//~CNVBlackBox();
    CNVBlackBox(ULONG dBufStartPos , ULONG dBufEndPos);

	void NVRamSelAddr(UINT uAddr);
	void NVRamSelAddrOpti(UINT uAddr);
	void NVRamWrite(UINT uAddr, UCHAR uchData);
	void NVRamWriteOpti(UINT uAddr, UCHAR uchData);
	UCHAR NVRamRead(UINT uAddr);
	UCHAR NVRamReadOpti(UINT uAddr);
	void NVRamWrite(UCHAR uchData);
	UCHAR NVRamRead();
	USHORT NVRamReadUshort(UINT uAddr);
	void NVRamWriteUshort(UINT uAddr, USHORT u);
	ULONG NVRamReadUlong(UINT uAddr);
	void NVRamWriteUlong(UINT uAddr, ULONG u);

	// 生成黑匣子
    int Create(char* strFileName , ULONG dBufStartPos , ULONG dBufEndPos );
    int Create(char* strFileName , ULONG dBufStartPos , ULONG dBufEndPos ,CFile *pFile );

	// 清除黑匣子内所有内容
	void Clear();
    //重置黑匣子
	void Reset();
	// Save the data to a file
	void Save(LPCTSTR strFile);

	CNVBlackBox& operator << (BYTE ch);
	CNVBlackBox& operator << (char ch);
	CNVBlackBox& operator << (char* p);
	CNVBlackBox& operator << (int n);
	CNVBlackBox& operator << (USHORT u);
//    CNVBlackBox& operator << (LONG l);
	CNVBlackBox& operator << (float f);
    CNVBlackBox& operator << (struct tm *t);

	void NewRecord();
	void EndRecord();
	void RewriteCriticalData();
    ULONG CalCheckSum(ULONG uPos);
    ULONG CalCheckSumExt(ULONG uHeadPos,ULONG uTailPos);
	void ObtainBBName(UCHAR* p);
	void CreateBBName(char* strFileName);
	void ObtainBpBBName(UCHAR* p);
	void CreateBpBBName(char* strFileName);

    static void SetNVRamReadUshort(USHORT (*Proc)(ULONG));
    static void SetNVRamWriteUshort(void (*Proc)(ULONG,USHORT));

    static void SetNVRamReadUchar(UCHAR (*Proc)(ULONG));
    static void SetNVRamWriteUchar(void (*Proc)(ULONG,UCHAR));
};
#endif
