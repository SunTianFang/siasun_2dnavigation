/*
 * NvRamChannel.cpp
 *
 *  Created on: 2018-6-7
 *
 * Author: zhaozaixin & liuyu & sfe1012
 *
 */

#include "NvRamChannel.h"

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
  __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)


#ifdef   _MRC_LINUX32
    #define   NVRAM_PORT_BASE		0x20000000
    #define   MAP_SIZE              128UL
#endif

#ifdef _Borax_LINUX32

    #define AUDIO_REG_BASE   (0xB800E000)
    #define MAP_SIZE_BORAX          32*1048576 //0x4000 /*0x40000000  */
    #define SPI0_ADD_BASE	  (0xfff01000)

    #define WREN	0x06
    #define WRTE	0x02
    #define READ	0x03
    #define RDSR 	0x05
    #define WRSR 	0x01
    #define WRDI 	0x04

    #define SPI_STATUS_ROE_MSK              (0x8)
    #define SPI_STATUS_ROE_OFST             (3)
    #define SPI_STATUS_TOE_MSK              (0x10)
    #define SPI_STATUS_TOE_OFST             (4)
    #define SPI_STATUS_TMT_MSK              (0x20)
    #define SPI_STATUS_TMT_OFST             (5)
    #define SPI_STATUS_TRDY_MSK             (0x40)
    #define SPI_STATUS_TRDY_OFST            (6)
    #define SPI_STATUS_RRDY_MSK             (0x80)
    #define SPI_STATUS_RRDY_OFST            (7)
    #define SPI_STATUS_E_MSK                (0x100)
    #define SPI_STATUS_E_OFST               (8)

    #define SPI_REGS_BUFSIZE 1024

    #define ALT_STM_OFST        0xfc000000
    #define HW_REGS_BASE ( ALT_STM_OFST )
    #define HW_REGS_SPAN ( 0x04000000 )
    #define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
    #define ALT_LWFPGASLVS_OFST        0xff200000
    #define		BUFF_SZ		1024

    static int cs_mask = 0xffff;

    #define  ADDRESS_BOUNDAY_VALUE      524287  // 0~524287:524288

#endif

cNvRamChannel::cNvRamChannel()
{
    m_iFd = -1;
    m_pMapBase = NULL;
    m_MapLength = -1;
    m_bIsOpened = FALSE;
#ifdef _Borax_LINUX32
    m_pDataBase = NULL;
    cs_add_base = NULL;
#endif
}

cNvRamChannel::~cNvRamChannel()
{
    Close();
}

BOOL cNvRamChannel::Open()
{

#ifdef _MRC_LINUX32
    m_iFd = open("/dev/mem", O_RDWR | O_SYNC);
    if (m_iFd < 0)
    {
        cout<<("open(/dev/mem) failed.")<<endl;
        return FALSE;
    }

    m_MapLength = MAP_SIZE;

    m_pMapBase = (USHORT*)mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, m_iFd, NVRAM_PORT_BASE);

    if (m_pMapBase == MAP_FAILED)
    {
        cout<<"Memory mapped at address"<<endl;
        return FALSE;
    }
#endif


#ifdef _Borax_LINUX32

    m_iFd = open("/dev/mem", O_RDWR | O_SYNC);
    if (m_iFd < 0)
    {
        cout<<("open(/dev/mem) failed.")<<endl;
        return FALSE;
    }
    m_MapLength = /*MAP_SIZE_BORAX*/HW_REGS_SPAN;
    m_pMapBase = (unsigned char * )mmap(NULL, /*MAP_SIZE_BORAX*/HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, m_iFd,  HW_REGS_BASE );
    if( m_pMapBase == MAP_FAILED )
    {
        close( m_iFd );
        cout<<"Memory mapped at address"<<endl;
        return FALSE;
    }
    cs_add_base  =  m_pMapBase + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + 0 ) & ( unsigned long)( HW_REGS_MASK ) );
    m_pDataBase  =  m_pMapBase + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + 0x40 ) & ( unsigned long)( HW_REGS_MASK ) );

#endif

    m_bIsOpened = TRUE;

    if (m_iFd > 0) //Map Over ,Then can Close  /dev/mem
    {
       close(m_iFd);
       m_iFd = -1;
    }

    return TRUE;
}

BOOL cNvRamChannel::Close()
{
    if(m_bIsOpened)
    munmap((void*)m_pMapBase, m_MapLength);

    if (m_iFd > 0)
    close(m_iFd);


    m_iFd = -1;
    m_pMapBase = NULL;
    m_MapLength = -1;
    m_bIsOpened = FALSE;
#ifdef _Borax_LINUX32
    m_pDataBase = NULL;
    cs_add_base = NULL;
#endif

    return TRUE;
}

/*
 * Operation read and write interval
 */
void cNvRamChannel::DelayForNVRAM(void)
{
    int x = 0;
    for(x = 0;x < 100; x++)
    {
        ;
    }
}

//
//   Read the NVRAM.
//
USHORT cNvRamChannel::ReadNVRAM(ULONG dwAddr)
{
    USHORT uResult = -1;

#ifdef PLATFORM_WINDOWS_ARMV4I
    #ifdef _MRC_LINUX32
        if (!m_bIsOpened)
            return -1;

        m_NVRAMCritSec.Lock();

        DelayForNVRAM();

        m_pMapBase[1] = dwAddr&0xFFFF;
        m_pMapBase[3] = (dwAddr>>16)&0x03;
        if (dwAddr <= 0x3FFFF)
            uResult = m_pMapBase[0];
        else
            uResult = m_pMapBase[0x0c];

        m_NVRAMCritSec.Unlock();
    #endif

    #ifdef _Borax_LINUX32

        if (!m_bIsOpened)
            return -1;

        m_NVRAMCritSec.Lock();

        dwAddr = dwAddr*2;

        ULONG dwAddrTemp = 0;
        int   iChip = -1;

        union
        {
            USHORT u;
            BYTE uch[2];

        } DataBuf;

        if(dwAddr > ADDRESS_BOUNDAY_VALUE)
        {
            dwAddrTemp = (dwAddr / ADDRESS_BOUNDAY_VALUE)*ADDRESS_BOUNDAY_VALUE + dwAddr % ADDRESS_BOUNDAY_VALUE;
            iChip = 1;
        }
        else
        {
            dwAddrTemp = dwAddr;
            iChip = 0;
        }
       chip_read(iChip,dwAddrTemp,DataBuf.uch,2);

       uResult =  DataBuf.u;

       m_NVRAMCritSec.Unlock();

    #endif

#endif

    return uResult;
}

//
//   Write to the NVRAM.
//
void cNvRamChannel::WriteNVRAM(ULONG dwAddr, USHORT val) //dwAddr is ushort type , ever unit need two char
{
#ifdef PLATFORM_WINDOWS_ARMV4I
    #ifdef _MRC_LINUX32
        if (!m_bIsOpened)
            return;

        m_NVRAMCritSec.Lock();

        DelayForNVRAM();
        m_pMapBase[1] = dwAddr&0xFFFF;
        m_pMapBase[3] = (dwAddr>>16)&0x03;
        if (dwAddr <= 0x3FFFF)
            m_pMapBase[0] = val;
        else
            m_pMapBase[0x0c] = val;

        m_NVRAMCritSec.Unlock();
    #endif

   #ifdef _Borax_LINUX32

        if (!m_bIsOpened)
            return;

        m_NVRAMCritSec.Lock();

        ULONG dwAddrTemp = 0;
        int iChip = -1;

        dwAddr = dwAddr*2;  //dwAddrTemp is char unit

        union
        {
            USHORT u;
            BYTE uch[2];
        } DataBuf;
        DataBuf.u = val;


        if( dwAddr > ADDRESS_BOUNDAY_VALUE)
        {
//            dwAddrTemp = (dwAddr / ADDRESS_BOUNDAY_VALUE)*ADDRESS_BOUNDAY_VALUE + dwAddr % ADDRESS_BOUNDAY_VALUE;
            dwAddrTemp = (dwAddr - ADDRESS_BOUNDAY_VALUE);
            iChip = 1;
        }
        else
        {
            dwAddrTemp = dwAddr;
            iChip = 0;
        }
       chip_wirte(iChip,dwAddrTemp,DataBuf.uch,2);

       m_NVRAMCritSec.Unlock();

   #endif

#endif
}

void cNvRamChannel::set_pio(char val,int num)
{
    #ifdef _Borax_LINUX32
    if (val == 1)
    {
        cs_mask |= (1<<num);
    }
    else
    {
        cs_mask &= ~(1<<num);
    }
    *(uint32_t *)cs_add_base = cs_mask;
    #endif
}

void cNvRamChannel::cfg_cs(uint8_t cs_num,uint8_t val)
{
    #ifdef _Borax_LINUX32
    set_pio(val,cs_num);
    #endif
}


void cNvRamChannel::spi_send(int val)
{
    #ifdef _Borax_LINUX32
    *(volatile unsigned int *)(m_pDataBase + 4) = val;
    #endif
}

unsigned char cNvRamChannel::spi_read()
{
    #ifdef _Borax_LINUX32
    return (*(volatile unsigned char *)(m_pDataBase));
    #else
    return 0;
    #endif
}

int cNvRamChannel::spi_status(void)
{
    #ifdef _Borax_LINUX32
    return *(volatile unsigned int *)(m_pDataBase + 8);
    #else
    return -1;
    #endif
}

void cNvRamChannel::chip_wirte(int chip , unsigned int addr, unsigned char *wrptr,unsigned int len )
{
    #ifdef _Borax_LINUX32
        unsigned int i;
        char buf[4];

        buf[0] = WREN;
        cfg_cs(chip,0);
        while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
        spi_send(buf[0]);
        while (0 == (spi_status() & SPI_STATUS_TMT_MSK));
        cfg_cs(chip,1);

        buf[0] = WRTE;
        if (len > BUFF_SZ)
            len = BUFF_SZ;
        cfg_cs(chip,0);
        buf[1] = (uint8_t)(addr>>16);
        buf[2] = (uint8_t)(addr>>8);
        buf[3] = (uint8_t)(addr);
        for (i=0;i<4;i++)
        {
            while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
            spi_send(buf[i]);
        }
        for (i=0;i<len;i++)
        {
            while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
            spi_send(wrptr[i]);
        }
        while (0 == (spi_status() & SPI_STATUS_TMT_MSK));
        cfg_cs(chip,1);

    #endif
}

void cNvRamChannel::chip_read(int chip,unsigned int addr, unsigned char *rdptr, unsigned int len )
{
    #ifdef _Borax_LINUX32
        unsigned int i;
        char buf[4];

        buf[0] = READ;
        cfg_cs(chip,0);
        if (len > BUFF_SZ)
            len = BUFF_SZ;

        buf[1] = (uint8_t)(addr>>16);
        buf[2] = (uint8_t)(addr>>8);
        buf[3] = (uint8_t)(addr);
        for (i=0;i<4;i++)
        {
            while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
            spi_send(buf[i]);
        }
//        for (i=0;i<len;i++)
//        {
//            while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
//            spi_send(0x05);
//            rdptr[i] =  spi_read();
//        }
//        while (0 == (spi_status() & SPI_STATUS_RRDY_MSK));

        for (i=0;i<len;i++)
        {
            while( 0 == (spi_status() & SPI_STATUS_TRDY_MSK));
            spi_send(i);
            while (0 == (spi_status() & SPI_STATUS_RRDY_MSK));
            rdptr[i] =  spi_read();
        }

        cfg_cs(chip,1);
    #endif
}
