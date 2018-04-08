//<<< Use Configuration Wizard in Context Menu >>>

/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/main.c
 * @author  MCD Application Team
 * @version V1.5.0
 * @date    06-March-2015
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Lib_retarget_printf.h"
#include "clock.h"
#include "IO.h"
#include "CAN.h"
#include "lift.h"
#include "encoder.h"
#include "NAV350.h"
#include "usart.h"
#include <stdarg.h>
#include "iap.h"
#include "stm32f10x_flash.h"
#include "timer.h"
#include "upgrade.h"
#include "motor.h"
#include "EncoderNavigation.h"
/****************************************************************************
* 名    称：void InitSystem(void)
* 功    能：系统初始化
* 入口参数：无
* 出口参数：
****************************************************************************/


void InitSystem(void)
{
    Init_CLK ();
	SYS_Timer_Init(Period_1MS);
	Delay_ms(20000);
	
    InitAllIO();
	
    InitAllCAN();

	motor_init();

	Nav350CommInit();

	InitAllEncoder();
	
	EncoderNavigation();
	Reinit_Hight();
	
}


int main(void)
{
    extern u32  sysTick;
    extern u8   f_startDownload;
		bool upPowerFlag = 1;
    InitSystem ();
    
		FeedBackLoop();
    while (1)
    {
        PinBlink(O_LED_HEART, 1, 1,NULL);
				//if(upPowerFlag && motor_mode_get())
				{
					//upPowerFlag = 0;
					//Reinit_Hight();
				}
				//else if(upPowerFlag == 0)
				{
					Nav350LoopPro();
					TxHight();
				}
		    Timer_Check();
		    message_process();
			
//        if(f_startDownload)
//        {
//            f_startDownload = 0;
//            AppJumptoIap (STM32_FLASH_BASE);
//        }
//        {
//            static u32 lastTick = 0;
//            u32 delta_tick;
//            delta_tick = sysTick - lastTick;
//            lastTick = sysTick;
//            //SHOW_INT(delta_tick);
//        }
    }
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file,
                   uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     *  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    RTT_printf ("file=%s,line=%d \r\n", file, line);

    while (1)
    {
    }
}


#endif


/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//<<< end of configuration section >>>


