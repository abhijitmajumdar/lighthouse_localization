#ifndef _COMM_H
#define _COMM_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef USE_STDPERIPH_DRIVER
  #include <stm32f10x.h> // register defines
  #include <stm32f10x_conf.h> // standard peripheral library
#endif
#ifdef STM32CUBE
  #include <stm32f1xx.h> // register defines
#endif
#include <stdio.h> // io functions sprintf etc.
#include <string.h>
#include <stdlib.h> // malloc

// Configured UART1 on A9(TX),A10(RX)
// Can be used to update firmware(DFU)
// Supports 72MHz clock
#define UART_RX_BUFFER_SIZE 256

void uart_init(uint32_t baud, uint8_t use_dma);
void uart_send(char ch);
void uart_sends(const char* s);
uint8_t available();
void uart_gets(uint8_t* _buffer, uint8_t len);
uint8_t uart_getc();
void uart_deinit();

void uart_clear_line(uint8_t len);

#ifdef __cplusplus
}
#endif

#endif //_COMM_H
