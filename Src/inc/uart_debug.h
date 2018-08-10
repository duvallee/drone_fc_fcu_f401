/*
 *  File: uart_debug.h
 *
 * COPYRIGHT(c) 2018 MICROVISION Co., Ltd.
 *
*/


#ifndef __UART_DEBUG_H__
#define __UART_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif 

#if defined(UART_DEBUG_OUTPUT)

void uart_debug_init();
int uart_debug_put(unsigned char ch);

#endif   // UART_DEBUG_OUTPUT

#ifdef __cplusplus
}
#endif


#endif   // __UART_DEBUG_H__


