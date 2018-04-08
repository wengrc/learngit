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
 * 自定义类型
 ************************************************************************/


/***********************************************************************
 * 旗标定义
 ************************************************************************/


/***********************************************************************
 * 全局变量声明
 ************************************************************************/
extern int  sw_putchar;


/***********************************************************************
 * 全局函数声明
 ************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*printf重定向到RTT */
void  RTT_printf(char *fmt,
                 ...);


#ifdef __cplusplus
}
#endif

#endif /*_USART_H*/


