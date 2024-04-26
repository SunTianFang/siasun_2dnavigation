// MemFileMap.cpp : implementation file
//   Author: sfe1012
//   Date:   2018. 01. 10
//
#include "stdafx.h"
#include "MemFileMap.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include"Project.h"



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define handle_error(msg) \
    do { perror(msg); exit(EXIT_FAILURE); } while (0)


void err_exit(const char *err_msg)
{
    printf("error:%s\n", err_msg);
    exit(1);
}

/* 信号处理器 */
void signal_handler(int signum)
{
    if (signum == SIGSEGV)
        printf("\nSIGSEGV handler!!!\n");
    else if (signum == SIGBUS)
        printf("\nSIGBUS handler!!!\n");
    exit(1);
}


/////////////////////////////////////////////////////////////////////////////
// CMemFileMap

CMemFileMap::CMemFileMap()
{
  m_pBuffer = NULL;
  m_hMapFile = 0;
  m_hSynEvent = NULL ; 
}

/*
*函数介绍：创建或打开内存映射文件
*返回值：<=0 ,代表失败
*        >0,代表成功
*       -1 : 代表创建失败
*       -2 : 代表映射失败
*       -3 : 创建同步事件失败
*       -4 : 注册消息失败
*        1 : 代表创建成功，并且是新创建
*        2 : 代表打开成功，文件已存在
*/
int CMemFileMap::OpenFileMap(CString strMapName, DWORD dwMaxSize)
{

    CString strPathName = (LOG_FILE_PATH);
    strPathName += strMapName;

    int dwReturn = 1; //定义返回变量
    m_MaxSize = dwMaxSize;

    CloseFileMap();


//    /* 设置信号处理器 */
//    if (signal(SIGSEGV, signal_handler) == SIG_ERR)
//        err_exit("signal()");
//    if (signal(SIGBUS, signal_handler) == SIG_ERR)
//        err_exit("signal()");

    m_hMapFile = open(strPathName.c_str(), (O_RDWR));
    if (m_hMapFile < 0)
    {
       m_hMapFile = -1;
       return -1; // 代表创建失败
    }

    //extern void *mmap (void *__addr, size_t __len, int __prot,int __flags, int __fd, __off_t __offset)

    m_pBuffer = mmap(NULL, dwMaxSize, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_hMapFile , 0);

//    std::cout<<MAP_FAILED<<"   "<<getpagesize()<<std::endl;
//    std::cout<<m_pBuffer<<std::endl;

    if (MAP_FAILED!= m_pBuffer)
       dwReturn = 2; // 代表内存文件已打开
    else
    {
         handle_error("mmap");
         return -2;   // 映射文件的视图到进程的地址空间失败
    }
//add for Test
    if (m_hMapFile)
    {
        close(m_hMapFile);
        m_hMapFile = 0;
    }

    m_hSynEvent = CreateEvent(NULL, FALSE, TRUE, NULL);// _T("MEMFILE"));
    if (!m_hSynEvent)
    {
      munmap(m_pBuffer, m_MaxSize);
      close(m_hMapFile);
      m_pBuffer = NULL;
      m_hMapFile = 0;
      return -3;          // 创建同步事件失败
    }



  return dwReturn;
}

/*
*函数介绍：关闭内存映射文件
*/
void CMemFileMap::CloseFileMap()
{
	if (m_pBuffer)
	{
//      UnmapViewOfFile(m_pBuffer);
      munmap(m_pBuffer, m_MaxSize);
      m_pBuffer = NULL;
	}

	if (m_hMapFile)
	{
        close(m_hMapFile);
        m_hMapFile = 0;
	}

	if (m_hSynEvent)
	{
	   CloseHandle(m_hSynEvent);
	   m_hSynEvent = NULL;
	}
}

/*
*函数介绍：读取内存映射文件内容
*返回值：内存文件内容指针
*/
LPVOID CMemFileMap::GetBuffer()
{
  return m_pBuffer;
}

/*
*函数介绍：写入内存映射文件内容
*入口参数：buf :要写入的字符串数据指针
*出口参数：(无)
*返回值：-1 ：失败；1：成功
*/
DWORD CMemFileMap::WriteBuffer(LPCTSTR buf)
{
   if ((strlen(buf)) > (size_t)m_MaxSize)
      return -1;         //写入缓冲区，大于定义大小

  //同步写过程，等待同步信号，且等到同步信号，自动关闭信号
   if (sem_wait(m_hSynEvent) == WAIT_OBJECT_0)
      strcpy((LPCTSTR)m_pBuffer, buf);

   SetEvent(m_hSynEvent);
   return 1;
}


