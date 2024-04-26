//
//   The interface of class "CThreadHelper".
//

#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <fstream>
#include "Tools.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CThreadHelper".
class CThreadHelper
{
private:
    std::atomic_bool    m_aHaltThread;
    std::thread*        m_pThreadHlp;
    std::mutex          m_ThreadMutex;

private:
    void RunThread(unsigned int nCycleTime)
    {
        while(1){
            std::lock_guard<std::mutex> thread_lock(m_ThreadMutex);
            SupportRoutineProxy();
            usleep(nCycleTime * 1000);
            if(m_aHaltThread.load()){
                return;
            }
        }
    }

protected:
    virtual void SupportRoutineProxy() = 0;

public:
    CThreadHelper()
    {
        m_pThreadHlp = nullptr;
        m_aHaltThread = false;
    }

    ~CThreadHelper()
    {
        StopThread();
    }

    bool CreateThread(unsigned int nCycleTime, int nThreadPolicy = THREAD_PRIORITY_NORMAL)
    {
        if(m_pThreadHlp != nullptr){
            return true;
        }
        m_aHaltThread = false;
        m_pThreadHlp = new std::thread(&CThreadHelper::RunThread, this, nCycleTime);
        if(m_pThreadHlp != nullptr){
            return true;
        }
        else {
            return false;
        }
    }

    bool StopThread()
    {
        if(m_pThreadHlp != nullptr){
            m_aHaltThread = true;
            m_pThreadHlp->join();
            delete m_pThreadHlp;
            m_pThreadHlp = nullptr;
        }
        return true;
    }
};


//函数模版不单可以替换类型本身，还能替换类型的成员函数.
//注意：1、名称只能是RunThread，不能在指定模版参数的时候修改;
//     2、RunThread只能是public的，除非把thread_helper_t定义到CThreadHelper2的内部.
template <typename TYPE, void (TYPE::*RunThread)() >
void* thread_helper_t(void* param)    //线程启动函数，声明为函数模板
{
    TYPE* This = reinterpret_cast<TYPE*>(param);
    This->RunThread();
    return nullptr;
}

class CThreadHelper2
{
private:
    HANDLE          m_hKillThread;       // Handle of "Kill thread" event
    HANDLE          m_hThreadDead;       // Handle of "Thread dead" event
    pthread_t       m_ThreadId;
    std::mutex      m_ThreadMutex;
    unsigned int    m_nCycleTime;

protected:
    virtual void SupportRoutineProxy() = 0;

public:
    CThreadHelper2()
    {
        m_hKillThread = NULL;
        m_hThreadDead = NULL;
        m_ThreadId = 0;
        m_nCycleTime = 0;
    }

    ~CThreadHelper2()
    {
        StopThread();
    }

    bool CreateThread(unsigned int nCycleTime, int nThreadPolicy = THREAD_PRIORITY_NORMAL)
    {
        if(m_ThreadId != 0){
            return true;
        }
        // Init signal events
        m_hKillThread = CreateEvent(NULL, false, false, NULL);
        m_hThreadDead = CreateEvent(NULL, false, false, NULL);
        if (m_hKillThread == NULL || m_hThreadDead == NULL){
            return false;
        }

        m_nCycleTime = nCycleTime;
        // Start the support procedure
        pthread_attr_t attr;
        SetPthreadPriority(SCHED_RR ,nThreadPolicy, attr);
        if(pthread_create(&m_ThreadId, &attr, thread_helper_t<CThreadHelper2, &CThreadHelper2::RunThread>, reinterpret_cast<LPVOID>(this)) != 0)
        {
            std::cout<<"Creat CThreadHelper2 Pthread Failed"<<std::endl;
            return false;
        }
        else{
            std::cout<<"Creat CThreadHelper2 Pthread OK"<<std::endl;
        }
        pthread_attr_destroy(&attr);
        return true;
    }

    bool StopThread()
    {
        if (m_ThreadId == 0){
            return false;
        }

        SetEvent(m_hKillThread);
        WaitForSingleObject(m_hThreadDead, 5000);
        PthreadJoin(m_ThreadId);

        if (m_hKillThread != NULL){
            CloseHandle(m_hKillThread);
            m_hKillThread = NULL;
        }
        if (m_hThreadDead != NULL){
            CloseHandle(m_hThreadDead);
            m_hThreadDead = NULL;
        }

        m_ThreadId = 0;
        return true;
    }

    void RunThread()
    {
        while (WaitForSingleObject(m_hKillThread, 0) != WAIT_OBJECT_0){
            std::lock_guard<std::mutex> thread_lock(m_ThreadMutex);
            SupportRoutineProxy();
            usleep(m_nCycleTime * 1000);
        }

        SetEvent(m_hThreadDead);
        pthread_exit(NULL);
    }
};

