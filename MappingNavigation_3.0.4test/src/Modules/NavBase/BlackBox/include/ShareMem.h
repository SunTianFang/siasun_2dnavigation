//                                  - cShareMem.h-
//
//   The  class "cShareMem".
//
//   Author: sfe1012
//   Date:   2018. 6 . 1
//
#ifndef CSHAREMEM_H
#define CSHAREMEM_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <sys/ipc.h>
#include <sys/sem.h>
#include <signal.h>

class cShareMem
{
public:
    cShareMem();
public:
    void  *CreatShareMem(const char *__pathname, unsigned long uSize);
    int    GetShmid(){  return m_iShmid;    }
    void   *GetShareMemAddress(){   return m_pSharedMemory;     }
    void   CloseShareMem();
private:
    int    ignore_signal(void);
private:
    void   *m_pSharedMemory;
    int     m_iShmid;

};

#endif // CSHAREMEM_H
