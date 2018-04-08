#ifndef __LIB_RETARGET_PRINTF_H
#define __LIB_RETARGET_PRINTF_H

#include <stdio.h>

#define ERR_LOCATE( error_cnt, whereFunc,title) \
    RTT_printf ("#######  %d.%s:%s  %s ( %d )\r\n",error_cnt,#whereFunc,title, __FILE__, __LINE__)

#define SHOW_VOID(V) \
        RTT_printf ("####### %s   %s  ( %d )\r\n", #V, __FILE__, __LINE__)

#define SHOW_INT(V) \
    RTT_printf ("####### %s = %d   %s  ( %d )\r\n", #V, V, __FILE__, __LINE__)

#define SHOW_FLOAT(V) \
    RTT_printf ("####### %s = %f   %s  ( %d )\r\n", #V, V, __FILE__, __LINE__)


/***********************************************************************
 * �Զ�������
 ************************************************************************/


/***********************************************************************
 * ��궨��
 ************************************************************************/


/***********************************************************************
 * ȫ�ֱ�������
 ************************************************************************/
extern int  sw_putchar;


/***********************************************************************
 * ȫ�ֺ�������
 ************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*printf�ض���RTT */
void  RTT_printf(char *fmt,
                 ...);


#ifdef __cplusplus
}
#endif

#endif /*_USART_H*/


