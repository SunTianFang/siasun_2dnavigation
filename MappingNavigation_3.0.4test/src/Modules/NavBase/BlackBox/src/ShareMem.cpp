//                             - cShareMem.cpp-
//
//   The  class "cShareMem".
//
//   Author: sfe1012
//   Date:   2018. 6 . 1
//

#include "ShareMem.h"

cShareMem::cShareMem()
{
    m_iShmid = -1;
    m_pSharedMemory = NULL;
}

int cShareMem::ignore_signal(void)
{
    signal(SIGINT, SIG_IGN);
    signal(SIGSTOP, SIG_IGN);
    signal(SIGQUIT, SIG_IGN);
    signal(SIGABRT,SIG_IGN);
    return 0;
}

void *cShareMem::CreatShareMem(const char *__pathname, unsigned long uSize)
{
    ignore_signal(); /* 防止程序非正常退出 */
    /* 创建共享内存 */
    m_iShmid = shmget(ftok(__pathname, 'b'), uSize, 0666|IPC_CREAT);
    if (m_iShmid == -1)
    {
        perror("shmget failed");
    }
    else
    {
        //printf("Create shared-memory: %d\n",m_iShmid);
    }
    /* 将共享内存地址映射到当前进程地址空间 */
    m_pSharedMemory = shmat(m_iShmid, (void*)0, 0);
    if (m_pSharedMemory == (void*)-1)
    {
        perror("shmat failed");
    }
    memset(m_pSharedMemory,0,uSize);
    return m_pSharedMemory;
}

void cShareMem::CloseShareMem()
{
    /* 删除共享内存到当前进程地址空间中的映射 */
    if (shmdt(m_pSharedMemory) == 1)
    {
        perror("shmdt failed");
    }
}
