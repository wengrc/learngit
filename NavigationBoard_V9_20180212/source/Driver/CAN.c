#include "CRC.h"
#include "CAN.h"

//#include "led.h"
#include "iap.h"
#include "clock.h"
#include "Lib_retarget_printf.h"
#include "motor.h"


/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /*外设时钟设置*/
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_CAN1, ENABLE);

    /*IO设置*/
    GPIO_PinRemapConfig (GPIO_Remap1_CAN1, ENABLE);

    /* Configure CAN pin: RX */                      // PB8
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;   // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

    /* Configure CAN pin: TX */                      // PB9
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

}


/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config_(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_3);

    /*中断设置*/
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);
}


/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
    CAN_InitTypeDef  CAN_InitStructure;

    /************************CAN通信参数设置**********************************/
    /*CAN寄存器初始化*/
    CAN_DeInit (CAN1);
    CAN_StructInit (&CAN_InitStructure);

    /*CAN单元初始化*/
    CAN_InitStructure.CAN_TTCM      = DISABLE;         //MCR-TTCM  关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM      = ENABLE;          //MCR-ABOM  自动离线管理
    CAN_InitStructure.CAN_AWUM      = ENABLE;          //MCR-AWUM  使用自动唤醒模式
    CAN_InitStructure.CAN_NART      = DISABLE;         //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
    CAN_InitStructure.CAN_RFLM      = DISABLE;         //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文
    CAN_InitStructure.CAN_TXFP      = ENABLE;          //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal; //正常工作模式
    CAN_InitStructure.CAN_SJW       = CAN_SJW_2tq;     //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1       = CAN_BS1_5tq;//CAN_BS1_3tq;//     //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2       = CAN_BS2_6tq;//CAN_BS2_2tq;//    //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler = 3;               ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36M/(1+3+2)/6=1000kbps
    CAN_Init (CAN1, &CAN_InitStructure);
}


/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    /*CAN过滤器初始化*/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                     //过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode   = CAN_FilterMode_IdMask; //工作在标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale  = CAN_FilterScale_32bit; //过滤器位宽为单个32位。

    /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0;                //过滤器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0;                //过滤器低16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //过滤器被关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;           //使能过滤器
    CAN_FilterInit (&CAN_FilterInitStructure);

    /*CAN通信中断使能*/
    CAN_ITConfig (CAN1, CAN_IT_FMP0, ENABLE);
}


/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config_(void)
{
    CAN_GPIO_Config ();
    CAN_NVIC_Config_ ();
    CAN_Mode_Config ();
    CAN_Filter_Config ();
}


/**
 * @brief  This function Initial all can bus
 * @param  None
 * @retval None
 */

void InitAllCAN(void)
{
#if 0
    struct CANBUSx  CANBUS1 =
    {
        CAN1,
        CAN1_RX1_IRQn,
        GPIOA,
        GPIO_Pin_12,
        GPIOA,
        GPIO_Pin_11,
        CAN_BS1_2tq,
        CAN_BS2_3tq,
        7
    };
    CAN_Config (&CANBUS1);
#else
    CAN_Config_ ();
#endif
}


/**
 * @brief  This function handles CAN1 RX0 request.
 * @param  None
 * @retval None
 */

u8  f_startDownload = 0;


void USB_LP_CAN1_RX0_IRQHandler(void)
{
//  extern fifo  fifo_from_linux;
    CanRxMsg  canRxMsgBuf;

#define START_DOWNLOAD   (canRxMsgBuf.Data[0x00] == 0XFF \
                          && canRxMsgBuf.Data[0x01] == 0XFF)

#define FINISH_DOWNLOAD  (canRxMsgBuf.Data[0x00] == 0XEE \
                          && canRxMsgBuf.Data[0x01] == 0XEE)

    // int          i;

    //CAN_ClearITPendingBit ( CAN1, CAN_IT_FMP0 );
    CAN_Receive (CAN1, CAN_FIFO0, &canRxMsgBuf);

    if (canRxMsgBuf.IDE == CAN_Id_Extended)
    {
        if (canRxMsgBuf.RTR == CAN_RTR_Data)
        {
            switch (canRxMsgBuf.ExtId)
            {

            case 0X10:

                if (START_DOWNLOAD && canRxMsgBuf.Data[2] == 0X01)
                {
                    f_startDownload = 1; //准备开始下载
                }

                if (FINISH_DOWNLOAD && canRxMsgBuf.Data[2] == 0X01)
                {
                }
                break;

            default:
                message_save (&canRxMsgBuf);
                break;
            }
        }
    }

    if (canRxMsgBuf.IDE == CAN_Id_Standard)
    {
        if (canRxMsgBuf.RTR == CAN_RTR_Data)
        {
            switch (canRxMsgBuf.StdId)
            {
            case 0x0a:

                break;

            default:
                break;
            }
        }
    }

}


/**
 * @brief  This function handles CAN1 RX0 request.
 * @param  None
 * @retval None
 */

enum
{
    CMD_SET_EN_OR_DISABLE = 1, //1

};


#define CAN_TXMAILBOX_0  ( (uint8_t)0x00 )
#define CAN_TXMAILBOX_1  ( (uint8_t)0x01 )
#define CAN_TXMAILBOX_2  ( (uint8_t)0x02 )
#define UNTIL_CAN_TxStatus_Ok()                                                   \
    {                                                                             \
        int  cnt = 100;                                                          \
        while (CAN_TransmitStatus (CAN1, CAN_TXMAILBOX_0) != CAN_TxStatus_Ok      \
               || CAN_TransmitStatus (CAN1, CAN_TXMAILBOX_1) != CAN_TxStatus_Ok   \
               || CAN_TransmitStatus (CAN1, CAN_TXMAILBOX_2) != CAN_TxStatus_Ok)  \
        {                                                                         \
            int  t =  1;                                                          \
            PERIOD_BLOCK (1, &t, cnt--);                                          \
            if (cnt <= 0)                                                         \
            {                                                                     \
                break;                                                            \
            }                                                                     \
        }                                                                         \
        if (cnt <= 0)                                                             \
        {                                                                         \
            int  t =  1;                                                          \
            PERIOD_BLOCK ( 1000, &t, RTT_printf ("CAN1 transmit out time\r\n") ); \
            break;                                                                \
        }                                                                         \
    }
		
#define TX_CAN1()                                              \
    while (CAN_Transmit (CAN1, &tx) == CAN_TxStatus_NoMailBox) \
    {                                                          \
        UNTIL_CAN_TxStatus_Ok ();                              \
    }

void ExtId_TX_CAN1(uint32_t ID,
                   char *pbuf,
                   int size)
{
    CanTxMsg  tx =
    {
        0, 0, CAN_Id_Extended, CAN_RTR_DATA
    };
    int       i;
    int       DataSize = size;

    if (size > 8)
    {
        int16_t  crc = GetCrc16 (pbuf, size, 0);
        ExtId_TX_CAN1_frameHead (ID, crc, size);

    }
    tx.ExtId = ID;
    tx.DLC   = 0;

    for (i = 0; i < DataSize; i++)
    {
        tx.Data[i] = *pbuf;
        pbuf++;
        tx.DLC++;

        if (tx.DLC == 8)
        {
            TX_CAN1 ();
            tx.DLC    = 0;
            i        -= 8;
            DataSize -= 8;
        }

    }

    if (tx.DLC != 0)
    {
        TX_CAN1 ();
    }

    if (size > 8)
    {

        ExtId_TX_CAN1_frameTail (ID);

    }
}


void ExtId_TX_CAN1_frameHead(uint32_t id,
                             uint16_t crc,
                             int size)
{
    CanTxMsg  tx =
    {
        0, 0, CAN_Id_Extended, CAN_RTR_DATA
    };

    tx.ExtId   = id;
    tx.Data[0] = 0xfe;
    tx.Data[1] = 0xfe;
    tx.Data[2] = (size >> 8) & 0xff;
    tx.Data[3] = size & 0xff;
    tx.Data[4] = (crc >> 8) & 0xff;
    tx.Data[5] = crc & 0xff;
    tx.DLC     = 6;
    TX_CAN1 ();

}


void ExtId_TX_CAN1_frameTail(uint32_t id)
{
    CanTxMsg  tx =
    {
        0, 0, CAN_Id_Extended, CAN_RTR_DATA
    };


    {
        uint32_t  lastTick;
        lastTick = GetSysTick ();

        while (GetSysTick () - lastTick < 1)
            ;
    }

    tx.ExtId   = id;
    tx.Data[0] = 0xfe;
    tx.Data[1] = 0xfe;
    tx.DLC     = 2;
    TX_CAN1 ();

}


#define INTER_TIME  1


