/*
 * NvRamChannel.h
 *
 *  Created on: 2018-6-7
 *
 *  Author: zhaozaixin & liuyu & sfe1012
 */


#ifndef CNVRAMCHANNEL_H
#define CNVRAMCHANNEL_H


#include"Project.h"
#include"ZTypes.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <time.h>


class cNvRamChannel
{
    public:
        cNvRamChannel();
        ~cNvRamChannel();
    public:
        BOOL Open();
        BOOL Close();
        //Read from the NVRAM
        USHORT ReadNVRAM(ULONG dwAddr);
        //   Write to the NVRAM.
        void WriteNVRAM(ULONG dwAddr, USHORT val);
    private:
        void DelayForNVRAM(void);
        void set_pio(char val,int num);
        void cfg_cs(uint8_t cs_num,uint8_t val);
        void chip_wirte(int chip,unsigned int addr, unsigned char *wrptr,unsigned int len );
        void chip_read(int chip,unsigned int addr, unsigned char *rdptr, unsigned int len );
        void spi_send(int val);
        unsigned char spi_read();
        int spi_status(void);
    private:
        CCriticalSection m_NVRAMCritSec;

        int   m_iFd;

#ifdef _MRC_LINUX32
       volatile USHORT*  m_pMapBase;
#endif

#ifdef _Borax_LINUX32
       volatile unsigned char *cs_add_base;
       volatile unsigned char *m_pMapBase;
       volatile unsigned char *m_pDataBase;
#endif


        size_t  m_MapLength;

        BOOL  m_bIsOpened;
};

#endif // CNVRAMCHANNEL_H
