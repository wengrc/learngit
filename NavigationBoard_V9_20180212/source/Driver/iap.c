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


//����ջ����ַ
//addr:ջ����ַ
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

        jump2iap = (iapfun) * (vu32 *)(iapxaddr + 4); //�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)
        MSR_MSP (*(vu32 *)iapxaddr);                  //��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
        jump2iap ();                                  //��ת��APP.
    }
    else
    {
        RTT_printf ("iap program loss,please check, %#X\r\n", data);
    }
}




