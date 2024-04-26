//                             - TOOLS.H -
//
//   The interface of some commonly used mathematical functions.
//

#ifndef __Tools
#define __Tools

#include "ZTypes.h"

#define FLOAT_GATE      1e-5

//////////////////////////////////////////////////////////////////////////////

// Limit "x" within "[-LimitVal, LimitVal]"
template <class Type> Type Limit(Type x, Type LimitVal)
{
	if (x > LimitVal)
		return LimitVal;

	else if (x > -LimitVal)
		return x;

	else
		return -LimitVal;
}

// Caculate the square of a value
template <class Type> Type Square(Type x)
{
	return (Type)(x * x);
}

//
// Let "from" approaches "to" at the increament of "step".
//
// Note:
//    1. "from" and "to" can be any value, but "fStep" should be positive.
//    2. If "from" reaches "to", TRUE is returned.
//
template <class Type>
bool Approach(Type& from, Type to, Type step)
{
	if (from + step < to)
	{
		from += step;
		return false;
	}

	else if (from - step > to)
	{
		from -= step;
		return false;
	}

	else
	{
		from = to;
		return true;
	}
}
// Check if 2 floats are approximately equal
bool DllExport ApprEqual(float x, float y, float fGate = FLOAT_GATE);

// Swab the high/low bytes of the specified word
void DllExport SwabWord(unsigned short int &uWord);

// Search for "1" in the mask byte
short DllExport FindBit(unsigned short Mask, short nFromBit = 0);

// Generate mask byte for the specified bit
unsigned short DllExport FindMaskWord(short nBit);


DllExport void CharToHexStr(UCHAR chAscii, UCHAR *pchHex);
DllExport bool HexStrToChar(UCHAR *pchHex, UCHAR& chAscii);
DllExport BOOL HexStrToUSHORT(UCHAR *pchHex, USHORT& UData);
DllExport unsigned short LookUpHex(unsigned char ch);

#define BIT(x)                 (((unsigned short) 1) << (x))
#define TestBit(x, i)          ((x & BIT(i)) != 0)
//#define sign(x)                ((x>=0) ? 1 : -1)     // The sign of variable x

// Defines PI
#define PI                     ((float)3.14159265)
#define TO_DEGREE(x)           (x/PI*180.0f)
#define TO_RADIAN(x)           (x/180.0f*PI)

DllExport float FabsAngleDiff(float angle1, float angle2);
#ifdef _MFC_VER
int HexStrToInt(CString& str);
bool DecStrToInt(CString& str, int& i);
bool DecStrToFloat(CString& str, float& f);
CString FloatToStr(float f);
#endif


#ifndef _MFC_VER
//sfe1012 add
typedef struct _stTimer
{
    int iHour;
    int iMin;
    int iSec;

}stTime;

//replace windows GetTickCount
inline unsigned long long GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

inline void Sleep(int ms)
{
    usleep(ms * 1000);
//    struct timeval delay;
//    delay.tv_sec = 0;
//    delay.tv_usec = ms*1000;
//    select(0,NULL,NULL,NULL,&delay);
}

inline void TurnMsTimeToNormTime(const long long msec,stTime &stOutTime)
{
    long long sec = msec / 1000;
    long long min = sec / 60;
    sec -= min * 60;
    long long hour = min / 60;
    min -= hour*60;

    stOutTime.iHour = hour;
    stOutTime.iMin = min;
    stOutTime.iSec = sec;
}

//1，SCHED_OTHER 分时调度策略，
//2，SCHED_FIFO实时调度策略，先到先服务。一旦占用cpu则一直运行。一直运行直到有更高优先级任务到达或自己放弃
//3，SCHED_RR实时调度策略，时间片轮转。当进程的时间片用完，系统将重新分配时间片，并置于就绪队列尾。放在队列尾保证了所有具有相同优先级的RR任务的调度公平
//SCHED_OTHER是不支持优先级使用的，而SCHED_FIFO和SCHED_RR支持优先级的使用，他们分别为1和99，数值越大优先级越高。
inline void SetPthreadPriority(const int iSchedulingAlgorithms ,const int iPthreadPolicy, pthread_attr_t &PthreadAttr)
{
    struct sched_param param;

    if(pthread_attr_init(&PthreadAttr) != 0)
    {
        std::cout<<"pthread_attr_init failed"<<std::endl;
    }

    if(pthread_attr_setschedpolicy(&PthreadAttr, iSchedulingAlgorithms) != 0)
    {
        std::cout<<"pthread_attr_setschedpolicy failed"<<std::endl;
    }

    param.sched_priority = iPthreadPolicy;

    if(pthread_attr_setschedparam(&PthreadAttr, &param) != 0)
    {
       std::cout<<"pthread_attr_setschedparam failed"<<std::endl;
    }
}

//默认bIsAutoRecoveryResources = true 情况下 不需要pthread_join进行线程资源回收
inline int SiaSun_AfxBeginThread(void *(*__start_routine) (void *),
                         void *__restrict __arg,
                         pthread_t *__restrict __newthread,
                         const int iPthreadPolicy = THREAD_PRIORITY_NORMAL,
                         const int iSchedulingAlgorithms = SCHED_RR,
                         const bool bIsAutoRecoveryResources = true
                         )
{
    //On success, pthread_create() returns 0; on error,
    //it returns an error number, and the contents of *thread are undefined.
    struct sched_param param;
    pthread_attr_t PthreadAttr ;

    if(pthread_attr_init(&PthreadAttr) != 0)
    {
        std::cout<<"pthread_attr_init failed"<<std::endl;
    }
    if(bIsAutoRecoveryResources) {
        if(pthread_attr_setdetachstate(&PthreadAttr, PTHREAD_CREATE_DETACHED) !=0 )//设置线程属性 由系统释放线程资源
        {
           std::cout<<"pthread_attr_setdetachstate failed"<<std::endl;
        }
    }
    if(pthread_attr_setschedpolicy(&PthreadAttr, iSchedulingAlgorithms) != 0)
    {
        std::cout<<"pthread_attr_setschedpolicy failed"<<std::endl;
    }
    param.sched_priority = iPthreadPolicy;
    if(pthread_attr_setschedparam(&PthreadAttr, &param) != 0)
    {
       std::cout<<"pthread_attr_setschedparam failed"<<std::endl;
    }
    int iRet = -1;
    iRet = pthread_create(__newthread,&PthreadAttr,__start_routine,__arg);
    if( iRet == EAGAIN)
    {
        std::cout<<"pthread_create failed , Insufficient resources to create another thread"<<std::endl;
    }
    else if(iRet == EINVAL)
    {
        std::cout<<"pthread_create failed , Invalid settings in attr"<<std::endl;
    }
    else if(iRet == EPERM)
    {
        std::cout<<"pthread_create failed , No permission to set the scheduling policy and parameters specified in attr"<<std::endl;

    }
    else if(iRet == ENOMEM)
    {
        std::cout<<"pthread_create failed , Out of memory"<<std::endl;

    }
    pthread_attr_destroy(&PthreadAttr);
    return iRet;
}

inline int WaitForSingleObject(sem_t* pInPutSem,int iMs)
{
    if(pInPutSem != NULL)
    {
        int iSec = iMs / 1000;
        int iNSec = (iMs%1000)*1000*1000;

        struct timespec ts;

        if ( clock_gettime( CLOCK_REALTIME,&ts ) < 0 )
            return -1;

        ts.tv_sec  += iSec;
        ts.tv_nsec += iNSec;

        ts.tv_sec += ts.tv_nsec/1000000000; //Nanoseconds [0 .. 999999999]
        ts.tv_nsec = ts.tv_nsec%1000000000;

        return sem_timedwait(pInPutSem,&ts);
        // return 0 on success; on error, the value of the semaphore is left unchanged, -1 is returned, and errno is set to indicate the error
    }
   return -1;
}

template <class Type>  sem_t* CreateEvent(Type bA, BOOL __pshared, BOOL __value, Type  bB)
{
   bA = 0;
   bB = 0;
       sem_t*  sem_tTemp  = new  sem_t;
       sem_init (sem_tTemp,  __pshared,  __value);
       return  sem_tTemp;
}

inline char * _tcsncpy(char *dest, const char *src, size_t n)
{
    return strncpy(dest,src, n);
}
inline char * _tcsncpy(char *dest, string src, size_t n)
{
    return strncpy(dest,src.c_str(), n);
}
inline char * _tcscpy(char *dest, const char *src)
{
        return strcpy(dest, src);
}
inline char * _tcscpy(char *dest, string src)
{
        return strcpy(dest, src.c_str());
}

inline void SecureZeroMemory(void * in, unsigned long size)
{
    memset(in, 0,size);
}

inline struct tm * GetCurrentTime()
{
   time_t lTime = 0;
   time_t tRetTime;
   tRetTime = time( &lTime );
   struct tm * lpstLocalTime = NULL;
   if(tRetTime)
   {
     lpstLocalTime = localtime( &lTime );
   }
   return lpstLocalTime;
}
inline void CloseHandle(sem_t *__sem)
{
    if(__sem)
    {
        sem_destroy(__sem);
//        delete __sem;
//        __sem = NULL;
    }
}
inline void SetEvent(sem_t *__sem)
{
    if(__sem)
    {
        sem_post(__sem);
    }
}
inline int PthreadExit()
{
     pthread_exit(NULL);
}
inline int PthreadJoin(pthread_t ThreadNum)
{
    void    *thrd_ret;
    return  pthread_join(ThreadNum, &thrd_ret);
}

//ntp时间戳转utc时间戳 返回ms
#define  SECOND_70_YEAR (60 * 60 * 24 * (365 * 70 + 17))
inline uint64_t NTPToUTC(uint64_t senconds, uint32_t picoseconds)
{
    senconds -= SECOND_70_YEAR;
    //ntp时间戳低32位单位pow(10,12) / pow(2,32) = 232皮秒
    uint32_t millsenconds = picoseconds * (pow(10,12) / pow(2,32)) / pow(10,9);

    return (senconds*1000 + millsenconds);
}

//获取当前系统时间ms
inline unsigned long long GetRealTimeTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

#endif// LINUX_PLATFORM_USING




#endif
