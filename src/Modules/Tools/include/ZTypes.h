//                               - ZTYPES.H -
//
//   Definition of macros that will be commonly used in programming.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 26
//

#if !defined __ZTYPES
#define __ZTYPES

//#define  _LINUX64

#ifndef _MSC_VER
#include<iostream>
#include<algorithm> //STL  algorithm 算法
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include<wchar.h>
#include<assert.h>
#include<pthread.h>
#include<semaphore.h>
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<time.h>
#include <math.h>
#include<sys/time.h>
#include <string.h>
#include<stdlib.h>
#include <arpa/inet.h>
#include<vector>
#include<deque>
#include<pthread.h>
using namespace std;
#endif

#define CP_CHINESE_PRC 936    // Code page of Chinese (P.R.C.)

// Defines data types
#ifndef _MFC_VER    // 如果没有MFC支持
typedef unsigned char UCHAR;
typedef unsigned short int BOOL;
typedef unsigned short int USHORT;
typedef short int SHORT;
#ifdef _LINUX64
    typedef    int                LONG;     //in ubuntu-64 long is 64bit
    typedef    unsigned int       ULONG;    //in ubuntu-64 long is 64bit
#else
    typedef    long               LONG;     //in mrc and gsrd arm linux long is 32bit
    typedef    unsigned long      ULONG;    //in mrc and gsrd arm linux long is 32bit
#endif
typedef unsigned int POSITION;
typedef unsigned long DWORD;
typedef unsigned short int WORD;
typedef unsigned char BYTE;
/*****************typedef**********************/
typedef int CFile;
typedef  char                CHAR;
typedef  char              _TCHAR;
typedef  char               TCHAR;
typedef char* 				LPCTSTR;
//typedef unsigned char       byte;
typedef float               FLOAT;
typedef int                 INT;
typedef unsigned int        UINT;
typedef  std::string             CString;
typedef  pthread_t         CWinThread;
typedef int                SOCKET;
typedef void *             LPVOID;
typedef struct sockaddr_in        SOCKADDR_IN ;
/*****************Define***************************/
#define IO_NOT_EXIST 0

#define SYSTEMTIME tm

#define GetLength size

//#define GetAt at

//#ifdef AGV_LINUX_DEBUG
//    #define ASSERT  assert
//    #define VERIFY  ASSERT
//#else
//    #define ASSERT
//    #define VERIFY
//#endif

#define _T

#ifndef WAIT_OBJECT_0
       #define WAIT_OBJECT_0  0
#endif

#ifndef TRUE
        #define TRUE 1
#endif

#ifndef FALSE
        #define FALSE 0
#endif

#ifndef __linux
 #define __linux 1
#endif

//define for socket
#ifndef SOCKET_ERROR
#define SOCKET_ERROR -1
#endif


#define inp(u)			0
#define outp(u, x)

#define  _stprintf  sprintf
#define  __max_      max
#define  __min_      min
#define  _tcsclen   strlen
#define  _tcslen    strlen
#define _tcscat wcscat

typedef  sem_t*            HANDLE;



#endif    // _MSC_VER

#ifndef _MSC_VER
#ifndef FALSE
#define FALSE ((BOOL)0)
#endif    // FALSE

#ifndef TRUE
#define TRUE ((BOOL)1)
#endif    // TRUE

#endif    // _MSC_VER

// Template support
#if defined _MSC_VER

#ifdef _MFC_VER
#define _ARCHIVE_SUPPORT
#endif

#if _MSC_VER >= 1000
#define _TEMPLATE_SUPPORT
#endif    // _MSC_VER >= 1000

#else    // _MSC_VER undefined

#ifdef __BCPLUSPLUS__
#define _TEMPLATE_SUPPORT
#endif
#endif

#ifdef _MSC_VER
#define DllExport    __declspec(dllexport)
#define DllImport    __declspec(dllimport)
#elif __GNUC__
#define DllExport     __attribute__((visibility("default")))
#define DllImport     __attribute__((visibility("default")))
#define _hypot  hypot
#endif

// WINCE dependant
#if defined _WIN32_WCE

extern "C" {
int __cdecl _inp(unsigned short);
int __cdecl _outp(unsigned short, int);
}

#define inp _inp
#define outp _outp

#pragma intrinsic(_inp)
#pragma intrinsic(_outp)

#define swab _swab
#define itoa _itoa

#elif defined _WIN32
#include <conio.h>
#define inp(u) 0
#define outp(u, x) 0
#endif

#define SHOW_DEBUG_MSG
#define ASSERT(x)
#define VERIFY(x) x



#ifndef _MSC_VER

#define VISIT_FOR_EACH_STL(iter, type, container)	\
        for(type::iterator (iter) = container.begin(); (iter) != container.end(); ++(iter))

#define VISIT_FOR_EACH_STL_CONST(iter, type, container)	\
        for(type::const_iterator (iter) = container.begin(); (iter) != container.end(); ++(iter))

#define FUNCTION_FAIL(function, ret, res)	\
        if ((ret) == (function)) {	\
        return (res);	\
        }

#define SAFE_DEL(x)	\
        if ((x)) {	\
        delete (x);	\
        (x) = NULL;	\
        }

#define SAFE_DEL_ARRAY(x)	\
        if ((x)) {	\
        delete [] (x);	\
        (x) = NULL;	\
        }

#define SAFE_REL(x)	\
        if ((x)) {			\
        (void)((x)->Release());	\
        (x) = NULL;		\
        }

#define SAFE_FREE(x)	\
        if ((x)) {			\
        free(x);	\
        (x) = NULL;		\
        }


class CCriticalSection
{
        public:
                CCriticalSection()
            {
                        pthread_mutex_init(&m_Mutex,NULL);
            }

                ~CCriticalSection()
                {
                        pthread_mutex_destroy(&m_Mutex);
                }

        private:
                pthread_mutex_t  m_Mutex;

        public:

                BOOL Unlock()
                {
                    if( pthread_mutex_unlock(&m_Mutex) == 0)
                        return TRUE;
                    return FALSE;
                }

                BOOL Lock()
                {
                    if(pthread_mutex_lock(&m_Mutex) == 0)
                        return TRUE;
                    return FALSE;
                }
};
/*************************Set pthrea priority*******************************************/
#ifndef PTHREAD_PRIORITY
    #define THREAD_PRIORITY_LOWEST          1
    #define THREAD_PRIORITY_BELOW_NORMAL    (THREAD_PRIORITY_LOWEST+1)
    #define THREAD_PRIORITY_NORMAL          96
    #define THREAD_PRIORITY_HIGHEST         99
    #define THREAD_PRIORITY_ABOVE_NORMAL    (THREAD_PRIORITY_HIGHEST-1)
#endif
/*************************************************************************************/
#endif //_MSC_VER


#endif    // __ZTYPES
