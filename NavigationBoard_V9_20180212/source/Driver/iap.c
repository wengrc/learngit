#include "iap.h"
#include "stdio.h"

#include "Lib_retarget_printf.h"
//#include "FIFO.h"
#include "Lib_IO.h"
#include "stm32f10x_can.h"
#include "stm32f10x_flash.h"
#include "upgrade.h"

void  USART1_printf(char *fmt,
                    ...);

typedef  void (*iapfun)(void); 


//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr)
{
    MSR  MSP, r0 //set Main Stack value
    BX r14
}


 iapfun  jump2iap;

void AppJumptoIap(u32 iapxaddr)
{
    int  data = (*(vu32 *)iapxaddr) & 0x2FFC0000;

    if (data == 0x20000000)
    {
        RTT_printf ("ok\r\n");

        jump2iap = (iapfun) * (vu32 *)(iapxaddr + 4); //用户代码区第二个字为程序开始地址(复位地址)
        MSR_MSP (*(vu32 *)iapxaddr);                  //初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
        jump2iap ();                                  //跳转到APP.
    }
    else
    {
        RTT_printf ("iap program loss,please check, %#X\r\n", data);
    }
}




