#ifndef _zf_board_h
#define _zf_board_h

#include "common.h"
#include "stdio.h"
#include "ch32v10x.h"
#include "zf_uart.h"


#define PRINTF_ENABLE           1                   //printfʹ��

#define DEBUG_UART              UART_3              //DEBUG����
#define DEBUG_UART_BAUD         115200              //DEBUG���ڲ�����
#define DEBUG_UART_TX_PIN       UART3_TX_C10         //DEBUG����TX����
#define DEBUG_UART_RX_PIN       UART3_RX_C11        //DEBUG����RX����

void board_init(void);




#endif
