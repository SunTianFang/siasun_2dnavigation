/******************************************************************************/
/*               #include������Ϊ��׼��ͷ�ļ����Ǳ�׼��ͷ�ļ���               */
/******************************************************************************/
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

//#include "include/commonDefine.h"

#include "mapping_file_opt.h"

/*
 * ����DPRINT�꣬��#define DEBUGʱ��������printf��������������Ϣ
 * ���ʱ�������������ļ��������ļ��У��Լ����ں�������Ϣ��
 */
#ifdef DEBUG
#define DPRINT(fmt,args...) \
    {\
        printf("%s(L:%d):%s:\t",__FILE__,__LINE__,__func__);\
        printf(fmt, ##args);\
    }
#else
#define DPRINT(fmt,args...) ;
#endif

int initCmpFile( PTCMP_FILE_DEVICE ptMappingFileDev )
{
    int nRtValue = FILECMP_SUCCESS;
    int nSeekPos = 0;
    //int fd;
    char tmp = 0;
    void *p_map = NULL;
    int nSize = 0;

    /* �ж���� */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /* ���ļ� */
    ptMappingFileDev ->nFd = open( ptMappingFileDev->cmpFileName, O_RDWR | O_CREAT, NULL );
    if( ptMappingFileDev ->nFd < 0 ) {
        perror( "open file failed " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * lseek �� write ��Ϊ�˵��ļ�����ָ������ʱ�����ļ��������
     */
    nSeekPos = ptMappingFileDev->len - 1;
    lseek( ptMappingFileDev ->nFd, nSeekPos, SEEK_SET );
    
    nSize = read( ptMappingFileDev ->nFd, &tmp, 1 );
    DPRINT( "read nSize is %d.\n", nSize );
    if( nSize < 1 ) { 
        write( ptMappingFileDev ->nFd, "", 1 );
    }
    DPRINT( "mappingFile %s's len is now 0x%X.\n", 
            ptMappingFileDev->cmpFileName, 
            ptMappingFileDev->len );

    /*
     * �����ļ�ӳ��
     */
    DPRINT( "mmap bgn.\n" ); 
    p_map = (char *)mmap( NULL, ptMappingFileDev->len, PROT_READ | PROT_WRITE, MAP_SHARED, ptMappingFileDev ->nFd, 0 );
    if( (void *)-1 == p_map ) {
        perror( "mmap failed " );
        nRtValue = FILECMP_FAILED;
        close( ptMappingFileDev ->nFd );
        goto EXIT_LABLE;
    } else {
        DPRINT( "p_map: 0x%X.\n", (unsigned int)p_map );
        ptMappingFileDev->pMappingAddr = p_map;
    }

EXIT_LABLE:
    return nRtValue;
}

int uninitCmpFile( PTCMP_FILE_DEVICE ptMappingFileDev )
{
    int nRtValue = FILECMP_SUCCESS;

    /*
     * ��Чָ��
     */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * munmapʧ��
     */
    if( 0 != munmap(ptMappingFileDev->pMappingAddr, 
                ptMappingFileDev->len) ) {
        perror( "munmap failed " );
        nRtValue = FILECMP_FAILED;
        //goto EXIT_LABLE;
    }
    close( ptMappingFileDev ->nFd );

EXIT_LABLE:
    return nRtValue;
}

int syncCmpFile( PTCMP_FILE_DEVICE ptMappingFileDev )
{
    int nRtValue = FILECMP_SUCCESS;

    /*
     * ��Чָ��
     */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * ˢ���ļ�
     */
    if( 0 != msync( ptMappingFileDev->pMappingAddr,
                ptMappingFileDev->len,
                MS_SYNC ) ) {
        perror( "msync failed " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

EXIT_LABLE:
    return nRtValue;
}
