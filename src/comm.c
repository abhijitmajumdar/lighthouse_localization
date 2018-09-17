#include "comm.h"

static uint16_t uart_rx_buffer_idx = 0;
static uint16_t uart_rx_buffer_idx_pointer = 0;
static uint8_t uart_use_dma = 0;
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

#ifdef __cplusplus
 extern "C" {
#endif
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      uart_rx_buffer[uart_rx_buffer_idx_pointer++] = USART_ReceiveData(USART1);
      if (uart_rx_buffer_idx_pointer==(UART_RX_BUFFER_SIZE-1)) uart_rx_buffer_idx_pointer=0;
    }
}
#ifdef __cplusplus
}
#endif

void uart_init(uint32_t baud, uint8_t use_dma)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
  uart_use_dma = use_dma;
  if (uart_use_dma){
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = UART_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  }
  else{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  if (uart_use_dma){
    uart_rx_buffer_idx = DMA_GetCurrDataCounter(DMA1_Channel5);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
  }
  else{
    uart_rx_buffer_idx = 0;
    uart_rx_buffer_idx_pointer = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  }
  USART_Cmd(USART1, ENABLE);
}

void uart_send(char ch)
{
  while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = (uint8_t)ch & USART_DR_DR;
}

void uart_sends(const char* s)
{
  int i=0;
  while(*(s+i)){
    uart_send((char)*(s+i));
    i++;
  }
}

uint8_t available()
{
  int idxs = uart_use_dma?(uart_rx_buffer_idx-DMA_GetCurrDataCounter(DMA1_Channel5)):(uart_rx_buffer_idx_pointer-uart_rx_buffer_idx);
  return (idxs>=0?idxs:UART_RX_BUFFER_SIZE+idxs);
}

void uart_gets(uint8_t* _buffer, uint8_t len)
{
  while(available()<len);
  uint8_t _idx=0;
  if(uart_use_dma){
    while(len--){
      _buffer[_idx++] = uart_rx_buffer[UART_RX_BUFFER_SIZE-uart_rx_buffer_idx];
      uart_rx_buffer_idx--;
      if (uart_rx_buffer_idx==0) uart_rx_buffer_idx=UART_RX_BUFFER_SIZE;
    }
  }
  else{
    while(len--){
      _buffer[_idx++] = uart_rx_buffer[uart_rx_buffer_idx];
      uart_rx_buffer_idx++;
      if (uart_rx_buffer_idx==UART_RX_BUFFER_SIZE) uart_rx_buffer_idx=0;
    }
  }
  _buffer[_idx++] = '\0';
}

uint8_t uart_getc()
{
  uint8_t rxd;
  while(available()<1);
  if (uart_use_dma){
     rxd = uart_rx_buffer[UART_RX_BUFFER_SIZE-uart_rx_buffer_idx];
     uart_rx_buffer_idx--;
     if (uart_rx_buffer_idx==0) uart_rx_buffer_idx=UART_RX_BUFFER_SIZE;
  }
  else{
    rxd = uart_rx_buffer[uart_rx_buffer_idx];
    uart_rx_buffer_idx++;
    if (uart_rx_buffer_idx==UART_RX_BUFFER_SIZE) uart_rx_buffer_idx=0;
  }
  return rxd;
}

void uart_deinit()
{
  USART_Cmd(USART1, DISABLE);
}




// Higher level functions

void uart_clear_line(uint8_t len)
{
  while(len--){
    uart_send(' ');
  }
  uart_send('\r');
}
