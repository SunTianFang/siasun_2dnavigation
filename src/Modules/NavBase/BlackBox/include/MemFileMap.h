//                              - MEMFILEMAP.H -

//   The interface of class "CMemFileMap".
//   Author: sfe1012
//   Date:   2018. 01. 10

//


#ifndef __MEMFILEMAP
#define __MEMFILEMAP

#include "Tools.h"

/////////////////////////////////////////////////////////////////////
//   The interface of class "CMemFileMap".
class CMemFileMap
{
protected:
    int m_hMapFile;       // 内存映射文件句柄
    LPVOID m_pBuffer;        // 内存文件映射数据句柄
	HANDLE m_hSynEvent;      // 写内存文件时，同步事件句柄

public:
	DWORD   m_MaxSize;       // 内存文件大小

public:
	CMemFileMap();

	// 创建或打开内存映射文件
	int OpenFileMap(CString strMapName, DWORD dwMaxSize);

	// 关闭映射文件
	void CloseFileMap();

	// 读取内存文件信息
	LPVOID GetBuffer();

	// 写入内存文件信息
	DWORD WriteBuffer(LPCTSTR buf);
};

#endif
