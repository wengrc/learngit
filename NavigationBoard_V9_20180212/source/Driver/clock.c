

#include "clock.h"
#include "Lib_retarget_printf.h"
/**
 * @brief
 * @param  None
 * @retval None.
 */
#define SYS_TICK_MS (SystemCoreClock / 1000)
void Init_CLK(void)
{

    {
        /* 配置为HSE
         * HSE启动
         * 等待HSE启动成功*/
        ErrorStatus  HSEStartUpStatus;
        RCC_HSEConfig (RCC_HSE_ON);
        HSEStartUpStatus = RCC_WaitForHSEStartUp ();

        while (HSEStartUpStatus == ERROR)
            ;

    }


    {
        /* 配置PLL时钟:
         *  1.PLLSource:  PLLSOURCE RCC_PLLSource_HSE or RCC_PLLSource_HSI
         *  2.M:          0 ~ 63, M= HSE/VCO_IN,当VCO_IN为2MHZ抖动最小
         *  3.P:          2,4,6,8
         *  4.N:          192 ~ 432(Mhz),VCO_OUT = VCO_IN ×N也必须这个范围
         *  5.Q:          4 ~ 15
         * 启动main PLL
         * 等待配置成功  */

        const u32  PLLSOURCE = RCC_PLLSource_HSE_Div1;
//        const u32  VCO_IN    = 1000000;
//        const u32  M         = (HSE_VALUE / VCO_IN);
//        const u32  P         = 2;
//        const u32  N         = (SystemCoreClock / VCO_IN) * P;
//        const u32  Q         = (N / 48);
//        RCC_PLLConfig (PLLSOURCE, M, N, P, Q);
        RCC_PLLConfig(PLLSOURCE,RCC_PLLMul_9);
        RCC_PLLCmd (ENABLE);

        while (RCC_GetFlagStatus (RCC_FLAG_PLLRDY) == RESET)
            ;

    }

    {
        /* 选择时钟源
         * 判断时钟源:PLL=0X08,HSE=0X04,HSI=0X00*/
        RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

        while (RCC_GetSYSCLKSource () != 0x08)
            ;

        //    RCC->CR &= (uint32_t)0xfffffffE;
        //    RCC_HSICmd ( DISABLE );
    }


    {
        /* 配置AHB时钟分频
         * 配置APB2(高速外设)分频
         * 配置APB2(低速外设)分频*/
        RCC_HCLKConfig (RCC_SYSCLK_Div1);
        RCC_PCLK2Config (RCC_HCLK_Div2);
        RCC_PCLK1Config (RCC_HCLK_Div4);
    }

    if ( SysTick_Config (SYS_TICK_MS) )
    {
        /* Capture error */
        while (1)
            ;
    }
    
    {
        static RCC_ClocksTypeDef  RCC_Clocks;
        RCC_GetClocksFreq (&RCC_Clocks);
        RTT_printf (" HCLK = %d\r\n PCLK1 = %d\r\n PCLK2 = %d\r\n SYSCLK = %d\r\n ",
                    RCC_Clocks.HCLK_Frequency, RCC_Clocks.PCLK1_Frequency,
                    RCC_Clocks.PCLK2_Frequency, RCC_Clocks.SYSCLK_Frequency);
    }

}


/**
 * @brief  This function 1ms Tick 1
 * @param  None
 * @retval None
 */


extern u32  sysTick;
u32  sysTick_10us;
/*void SysTick_Handler(void)
{
    sysTick++;
}*/


u32 GetSysTick(void)
{
    return sysTick;
}


void SetSysTick(u32 tick)
{
    sysTick = tick;
}

void Delay_ms(u32 nms)
{
  u32 lastTick;
  lastTick = GetSysTick();
  while(GetSysTick() - lastTick < nms)
    ;
}

void Delay_us(u32 nus)
{
    u32  ticks = 0;
    u32  told, tnow, tcnt = 0;
    u32  reload = SysTick->LOAD; 
    ticks = nus * (SYS_TICK_MS / 1000);       
    tcnt  = 0;
    told  = SysTick->VAL;      

    while (1)
    {
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; 
            else
                tcnt += reload - tnow + told;
            told = tnow;

            if (tcnt >= ticks)
                break; 
        }
    }
    ;
}


