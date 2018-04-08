#ifndef __IO_H
#define __IO_H
#include "Lib_IO.h"

/*PIN OPERATION*/
#define PinLow(pin)     PinOutput (pin, 0)
#define PinHigh(pin)    PinOutput (pin, 1)

#define EN_BORDCARD()   PinHigh(O_BORDCARD_EN)
#define DIS_BORDCARD()  PinLow(O_BORDCARD_EN)

#define EN_SYS()        PinHigh (O_SYS_CTRL_EN)
#define DIS_SYS()       PinLow (O_SYS_CTRL_EN)
#define IS_SYS_EN()     (ReadPinOutput(O_SYS_CTRL_EN) == 1)

#define SELECT_AUTO()   PinHigh(O_MODE_0_MANU_1_AUTO)
#define SELECT_MANU()   PinLow(O_MODE_0_MANU_1_AUTO)
#define IS_AUTO()       (ReadPinOutput(O_MODE_0_MANU_1_AUTO) == 1)



#define IF_PIN_HIGH(I_PIN)    (ReadPinInput (I_PIN) == 1)

#define O_LED_HEART           GPIOA, pin0
#define O_BORDCARD_EN         GPIOC, pin13
#define O_SYS_CTRL_EN         GPIOE, pin8
#define O_CHARGE              GPIOB, pin12

#define I_TOP                 GPIOB, pin15
#define I_BOTTOM              GPIOB, pin14

/*CAN PORT*/
#define CAN_TX                GPIOB, pin9
#define CAN_RX                GPIOB, pin8

/*ENCODER PORT*/
#define ENC_CORD_A            GPIOA, pin6
#define ENC_CORD_B            GPIOA, pin7


void  InitAllIO(void);

#endif


