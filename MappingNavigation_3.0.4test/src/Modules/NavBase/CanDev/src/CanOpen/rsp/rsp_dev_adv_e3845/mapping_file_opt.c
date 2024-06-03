/******************************************************************************/
/*               #include（依次为标准库头文件、非标准库头文件）               */
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
 * 定义DPRINT宏，在#define DEBUG时，将类似printf输出，否则不输出信息
 * 输出时，包含了所在文件，所在文件行，以及所在函数等信息。
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

    /* 判断入参 */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /* 打开文件 */
    ptMappingFileDev ->nFd = open( ptMappingFileDev->cmpFileName, O_RDWR | O_CREAT, NULL );
    if( ptMappingFileDev ->nFd < 0 ) {
        perror( "open file failed " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * lseek 与 write 是为了当文件不足指定长度时，对文件进行填充
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
     * 进行文件映射
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
     * 无效指针
     */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * munmap失败
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
     * 无效指针
     */
    if( NULL == ptMappingFileDev ) {
        perror( "Invalid InputParam " );
        nRtValue = FILECMP_FAILED;
        goto EXIT_LABLE;
    }

    /*
     * 刷入文件
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
