#include "comm.h"
#include "utils.h"

#define BUFF_LEN 128
static uint16_t timer_capture_values_period[BUFF_LEN];
static uint16_t timer_capture_values_high[BUFF_LEN];
/*
  Example configuration
  TIM_PWM_CAPTURE_t configuration = {
    .nvic_irqchannel = TIM2_IRQn,
    .dma_ch1 = DMA1_Channel5,
    .dma_ch2 = DMA1_Channel7,
    .timx = TIM2,
    .timx_rcc = RCC_APB1Periph_TIM2,
    .gpio_rcc = RCC_APB2Periph_GPIOA,
    .gpio_pin = GPIO_Pin_0,
    .gpio_port = GPIOA,
    .resolution = 4,
    ._center = 4000,
    ._sync_min = 50,
    ._sync_max = 150,
    ._sweep_max = 40
  };
*/
typedef struct TIM_PWM_CAPTURE{
  uint8_t nvic_irqchannel; //TIM1_IRQn,TIM2_IRQn,TIM3_IRQn,TIM4_IRQn
  DMA_Channel_TypeDef* dma_ch1; //DMA1_Channel2,DMA1_Channel5,DMA1_Channel6,DMA1_Channel1
  DMA_Channel_TypeDef* dma_ch2; //DMA1_Channel3,DMA1_Channel7,NULL,DMA1_Channel4
  TIM_TypeDef* timx; //TIM1,TIM2,TIM3,TIM4
  uint32_t timx_rcc; // RCC_APB1Periph_TIM2/3/4,RCC_APB2Periph_TIM1
  uint32_t gpio_rcc; //RCC_APB2Periph_GPIOA,RCC_APB2Periph_GPIOB
  uint16_t gpio_pin; // GPIO_Pin_x
  GPIO_TypeDef* gpio_port; //GPIOA,GPIOB
  uint8_t resolution; // Divisor of 1uS
  int16_t _center; // 4000uS
  int16_t _sync_min; // 50uS
  int16_t _sync_max; // 150uS
  int16_t _sweep_max; // 40uS
} TIM_PWM_CAPTURE_t;

void init_timer_capture(TIM_PWM_CAPTURE_t &config)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = config.nvic_irqchannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  RCC_APB2PeriphClockCmd(config.gpio_rcc, ENABLE);
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  // GPIO_PinRemapConfig()
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = config.gpio_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(config.gpio_port, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(config.dma_ch1);
  DMA_DeInit(config.dma_ch2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(config.timx->CCR1);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)timer_capture_values_period;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BUFF_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(config.dma_ch1, &DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(config.timx->CCR2);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)timer_capture_values_high;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BUFF_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(config.dma_ch2, &DMA_InitStructure);

  DMA_Cmd(config.dma_ch1, ENABLE);
  DMA_Cmd(config.dma_ch2, ENABLE);

  RCC_APB1PeriphClockCmd(config.timx_rcc, ENABLE);
  TIM_PrescalerConfig(config.timx, (72/config.resolution)-1, TIM_PSCReloadMode_Immediate); //71
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  //TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_PWMIConfig(config.timx, &TIM_ICInitStructure);
  TIM_SelectInputTrigger(config.timx, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(config.timx, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(config.timx, TIM_MasterSlaveMode_Enable);

  TIM_DMACmd(config.timx, TIM_DMA_CC1, ENABLE);
  TIM_DMACmd(config.timx, TIM_DMA_CC2, ENABLE);

  TIM_Cmd(config.timx, ENABLE);

  // TIM_ITConfig(config.timx, TIM_IT_CC1, ENABLE);
  // TIM_ITConfig(config.timx, TIM_IT_CC2, ENABLE);

  config._center *= config.resolution;
  config._sync_min *= config.resolution;
  config._sync_max *= config.resolution;
  config._sweep_max *= config.resolution;
}


void uart_send_uint(uint16_t val)
{
  if(val>=10){
    uart_send_uint(val/10);
    val = val%10;
  }
  uart_send('0'+val);
}

void uart_send_int(int16_t val)
{
  if(val<0){
    uart_send('-');
    val = -val;
  }
  uart_send_uint(val);
}

void uart_send_binary(uint16_t val)
{
  if(val>=2){
    uart_send_binary(val/2);
    val = val%2;
  }
  uart_send('0'+val);
}

void uart_send_binary_complete(uint16_t val)
{
  for(int _l=0;_l<16;_l++){
    uint16_t v = val & (1<<(16-_l));
    uart_send('0'+((v==0)?0:1));
  }
}

typedef enum OOTX_STATE{
  STOPPED=0,
  INITALIZED,
  WAIT_FOR_FRAME,
  CHECK_FRAME,
  PROCESS_FRAME
} OOTX_STATE_t;
#define OOTX_BUF_LEN 64

int main()
{
  char *formatted_str = (char*)malloc(128 * sizeof(char));
  uint16_t i=0;
  uint16_t *tc_h = timer_capture_values_high;
  uint16_t *tc_p = timer_capture_values_period;
  int16_t x,y;
  int16_t buf_idx=0;
  int16_t idx_h,idx_p;

  OOTX_STATE_t state=INITALIZED;
  uint16_t ootx_buf[OOTX_BUF_LEN];
  uint8_t ootx_buf_index = 0;
  uint8_t bit_counter = 0;
  uint32_t preamble = 0xffffffff;
  uint8_t preamble_received = 0;
  uint8_t _debug=0;

  uart_init(115200,0);
  delay(1000);
  TIM_PWM_CAPTURE_t configuration = {
    .nvic_irqchannel = TIM4_IRQn,
    .dma_ch1 = DMA1_Channel1,
    .dma_ch2 = DMA1_Channel4,
    .timx = TIM4,
    .timx_rcc = RCC_APB1Periph_TIM4,
    .gpio_rcc = RCC_APB2Periph_GPIOB,
    .gpio_pin = GPIO_Pin_6,
    .gpio_port = GPIOB,
    .resolution = 4,
    ._center = 4000,
    ._sync_min = 50,
    ._sync_max = 150,
    ._sweep_max = 40
  };
  init_timer_capture(configuration);
  while(1){
    idx_p = BUFF_LEN-DMA_GetCurrDataCounter(configuration.dma_ch1)-buf_idx;
    idx_p = idx_p>=0?idx_p:BUFF_LEN+idx_p;
    idx_h = BUFF_LEN-DMA_GetCurrDataCounter(configuration.dma_ch2)-buf_idx;
    idx_h = idx_h>=0?idx_h:BUFF_LEN+idx_h;
    if (idx_h>1 & idx_p>1){
      status_led_activity();
      int16_t next_idx = buf_idx+1;
      if (next_idx==BUFF_LEN) next_idx=0;
      if (tc_h[buf_idx]<configuration._sync_max & tc_h[buf_idx]>configuration._sync_min) { // If valid sync pulse is found
        // 500 tics = 48uS
        // 1 uS = 500/48 tics
        // uint16_t sync_pulse_code = int((0.096*tc_h[buf_idx])-5); // Decode the sync pulse
        uint16_t sync_pulse_code = int((((48/configuration.resolution)*tc_h[buf_idx])-2750)/500); // Decode the sync pulse

        // Process position
        if (tc_h[next_idx]<configuration._sweep_max){ // If valid position is found after the sync pulse
          int16_t pos = configuration._center-tc_p[buf_idx]; // Relative angle from center (4000uS)
          if (pos<configuration._center & pos>-configuration._center){ // Double check if position is within range
            if (sync_pulse_code%2==0){ // This was a X sweep
              x = pos;
            }
            else{  // This was a Y sweep
              y = pos;
            }
            i++;
          }
          buf_idx = next_idx+1; // Processed a sync and a sweep pulse
        }
        else{
          buf_idx = next_idx; // Processed a sync pulse only
        }
        if (buf_idx==BUFF_LEN) buf_idx=0;


        // Process sync data
        sync_pulse_code = (sync_pulse_code&0x0002)>>1;
        // Lookout for preamble
        // When found change updating buffer to avoid overwrite and change state
        preamble <<= 1;
        preamble |= sync_pulse_code;
        if ((preamble & 0x0003ffff) == 0x00000001){
          preamble_received = 1;
        }
        switch (state) {
          case STOPPED:
            // Do nothing, save time by not processing OOTX frame
            break;
          case INITALIZED:
            if(preamble_received){
              ootx_buf_index = 0;
              bit_counter = 0;
              preamble_received = 0;
              state = WAIT_FOR_FRAME;
            }
            break;
          case WAIT_FOR_FRAME:
            // Push bits into the ootx buffer
            if(preamble_received){
              preamble_received = 0;
              state = CHECK_FRAME;
            }
            else if(bit_counter<16){
              ootx_buf[ootx_buf_index] = (ootx_buf[ootx_buf_index]<<1)|sync_pulse_code;
              bit_counter++;
            }
            else{
              bit_counter = 0;
              ootx_buf_index++;
              if(sync_pulse_code==0){ //Wrong sync bit
                if((preamble & 0x0001ffff)!=0){ // Wrong only if its not preamble being received
                  state = INITALIZED;
                  uart_sends("Error: Sync bit\r\n");
                }
              }
              if (ootx_buf_index==OOTX_BUF_LEN){ // Overflow
                state = INITALIZED;
                uart_sends("Error: Buffer overflow\r\n");
              }
            }
            break;
          case CHECK_FRAME:
            // Check payload length matchup
            // Check CRC
            state = PROCESS_FRAME;
            break;
          case PROCESS_FRAME:
            state = INITALIZED;
            uint16_t cm = ootx_buf[16]&0xff;
            uart_send_binary(cm);
            // uart_send_uint(ootx_buf_index);
            uart_sends("\r\nProcessing frame\r\n");
            _debug=1;
            break;
        }
      }
      else{
        // If not a valid sync pulse, skip to next
        buf_idx = next_idx;
      }
      // status_led_reset();
    }
    if (_debug){
      // for(int _l=0;_l<OOTX_BUF_LEN;_l++){
      //   uart_send_binary_complete(ootx_buf[_l]);
      // }
      for(int _l=0;_l<ootx_buf_index;_l++){
        uart_send_uint(ootx_buf[_l]);
        uart_send(',');
      }
      uart_sends("\r\n\n\n");
      _debug=0;
    }
    if(i%100==0){
      // uart_sends("\r                        \r");
      // uart_send('\r');
      // uart_send_int(buf_idx);
      // uart_send(',');
      // uart_send_int(idx_p);
      // uart_send(',');
      uart_sends(">");
      // uart_send_binary(preamble);
      // uart_sends("\r\n");
      uart_send(':');
      uart_send_int(x);
      uart_send(',');
      uart_send_int(y);
      // uart_sends("\r                        \r");
      // uart_send_binary(x);
      // uart_sends("\r                        \r");
      // uart_send_int(int(preamble_counter));
      // uart_sends("\r                        \r");
      // uart_send_binary(ootx_buf[ootx_buf_index]);
      // status_led_activity();
      // for(int _l=0;_l<OOTX_BUF_LEN;_l++){
      //   uart_send_binary_complete(ootx_buf[_l]);
      // }
      // uart_sends("\r\n\n\n");
      // uart_send_binary_complete(i);
      // uart_send_uint(i);
      uart_sends("\r\n");
      i++;
    }
    delay(1);
  }
}
