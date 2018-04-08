#include "key.h"
#include "Lib_retarget_printf.h"
#include "CAN.h"
#include "clock.h"

static u8   key[2] = { 0 };
void TxKey(void)
{
    static u32  lastTick;
    char        buf[8] = { 0 };

    if (GetSysTick () - lastTick < 100)
    {
        return;
    }
    lastTick = GetSysTick ();

    buf[0] = 0x42;
    buf[1] = key[0];
    buf[2] = key[1];
    ExtId_TX_CAN1 (CAN_ES1, buf, 3);
}


void keyTop(void)
{
    static u32  lastTick;

    if (GetSysTick () - lastTick > 1500)
    {
        lastTick = GetSysTick ();
        RTT_printf ("Trigger Urgent\r\n");
    }

    key[0] = 1;

}


void KeyBottom(void)
{
    static u32  lastTick;

    if (GetSysTick () - lastTick > 1500)
    {
        lastTick = GetSysTick ();
        RTT_printf ("Trigger Bottom\r\n");
    }
    key[1] = 1;
}


void        InitAllEncoder(void);

void Key(void) //protocol b5
{
    TxKey ();
    key[0] = 0;
    key[1] = 0;

    if ( ScanKey (I_BOTTOM, PRESSED) )
    {
        KeyBottom ();
    }

    if ( ScanKey (I_TOP, PRESSED) )
    {
        keyTop ();
    }
}


