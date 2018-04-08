#include "CRC.h"
#include "CAN.h"

//#include "led.h"
#include "iap.h"
#include "clock.h"
#include "Lib_retarget_printf.h"
#include "motor.h"


/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����,PB8�������룬PB9�������
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /*����ʱ������*/
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_CAN1, ENABLE);

    /*IO����*/
    GPIO_PinRemapConfig (GPIO_Remap1_CAN1, ENABLE);

    /* Configure CAN pin: RX */                      // PB8
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;   // ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

    /* Configure CAN pin: TX */                      // PB9
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

}


/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config_(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_3);

    /*�ж�����*/
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //�����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init (&NVIC_InitStructure);
}


/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{
    CAN_InitTypeDef  CAN_InitStructure;

    /************************CANͨ�Ų�������**********************************/
    /*CAN�Ĵ�����ʼ��*/
    CAN_DeInit (CAN1);
    CAN_StructInit (&CAN_InitStructure);

    /*CAN��Ԫ��ʼ��*/
    CAN_InitStructure.CAN_TTCM      = DISABLE;         //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_ABOM      = ENABLE;          //MCR-ABOM  �Զ����߹���
    CAN_InitStructure.CAN_AWUM      = ENABLE;          //MCR-AWUM  ʹ���Զ�����ģʽ
    CAN_InitStructure.CAN_NART      = DISABLE;         //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
    CAN_InitStructure.CAN_RFLM      = DISABLE;         //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���
    CAN_InitStructure.CAN_TXFP      = ENABLE;          //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ��
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal; //��������ģʽ
    CAN_InitStructure.CAN_SJW       = CAN_SJW_2tq;     //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1       = CAN_BS1_5tq;//CAN_BS1_3tq;//     //BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2       = CAN_BS2_6tq;//CAN_BS2_2tq;//    //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler = 3;               ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36M/(1+3+2)/6=1000kbps
    CAN_Init (CAN1, &CAN_InitStructure);
}


/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(void)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    /*CAN��������ʼ��*/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                     //��������0
    CAN_FilterInitStructure.CAN_FilterMode   = CAN_FilterMode_IdMask; //�����ڱ�ʶ������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale  = CAN_FilterScale_32bit; //������λ��Ϊ����32λ��

    /* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0;                //��������16λÿλ����ƥ��
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0;                //��������16λÿλ����ƥ��
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //��������������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;           //ʹ�ܹ�����
    CAN_FilterInit (&CAN_FilterInitStructure);

    /*CANͨ���ж�ʹ��*/
    CAN_ITConfig (CAN1, CAN_IT_FMP0, ENABLE);
}


/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
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
                    f_startDownload = 1; //׼����ʼ����
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


