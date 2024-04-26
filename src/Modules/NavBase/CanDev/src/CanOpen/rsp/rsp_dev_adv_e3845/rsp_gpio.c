/*<FUNC+>**********************************************************************
* 函数名称: rsp_gpio.c                                                             
* 功能描述:                                                              
* 输入参数:                                                              
* 输出参数:                                                              
* 返 回 值:                                                              
* 操作流程:                                                              
* 其它说明:                                                              
* 修改记录:                                                              
*    修改日期          版本号        修改人        修改内容                  
* -------------------------------------------------------------------------- 
*<FUNC->**********************************************************************/

/******************************************************************************/
/*               #include（依次为标准库头文件、非标准库头文件）               */
/******************************************************************************/
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//include for ioctl
#include <fcntl.h>

//include for ioctl
#include <sys/ioctl.h>

#include <sys/mman.h>

#include "rsp_gpio.h"

/******************************************************************************/
/*                                  全局变量                                  */
/******************************************************************************/

/******************************************************************************/
/* 标准接口                                                                   */
/******************************************************************************/

/******************************************************************************/
/*                                 全局宏定义                                 */
/******************************************************************************/

/******************************************************************************/
/*                              全局数据类型声明                              */
/******************************************************************************/

/******************************************************************************/
/*                                全局变量声明                                */
/******************************************************************************/

/******************************************************************************/
/*                                 函数的实现                                 */
/******************************************************************************/
int g_iGpioPhyAddr = 0xFED0E1F8;
int g_iGpioLogicAddr = 0;
int g_iGpioValue = 0;


static void *s_g_pGpioLogicAddr = NULL;
static bool l_ifInitialized = false;


static void _set_gpio_value( int64_t value )
{

    //TODO:
    assert( true == l_ifInitialized );

    *(volatile int64_t*)(s_g_pGpioLogicAddr + 0x21F8) = value; 
    //printf("\r set bit %d\n", value );
    //*(volatile int64_t*)(s_g_pGpioLogicAddr + 0x21F8) = 0x00;
    //printf("\r\n set bit 0");

}

int RSP_GPIO_Open( void )
{
    int nRtValue = RSP_RT_SUCCESS;

	int fbb = 0;

    if ((fbb = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("Error opening /dev/mem ");
        return RSP_RT_FAILED;
    }

    s_g_pGpioLogicAddr = mmap( NULL, 0x4000, PROT_READ | PROT_WRITE, MAP_SHARED,fbb, 0xFED0C000);

#if 1
    if( NULL == s_g_pGpioLogicAddr ) {
        printf( "nvram mmap failed " );
    } else {
        //printf("\r\n GPIO phy addr 0x%x logic addr 0x%p 0x%p value"
        //        , g_iGpioPhyAddr, s_g_pGpioLogicAddr, s_g_pGpioLogicAddr);
    }
#endif	

    //printf("\r\n Before Gpio Value %ld" , *(volatile int64_t*)(s_g_pGpioLogicAddr + 0x21F8));


    l_ifInitialized = true;


    return nRtValue;
}

void RSP_GPIO_Up( void )
{
    _set_gpio_value( 0x01 );
}

void RSP_GPIO_Down( void )
{
    _set_gpio_value( 0x00 );
}

int RSP_GPIO_Close( void )
{
    return RSP_RT_SUCCESS;
} 
