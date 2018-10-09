#include "comm.h"
#include "utils.h"
#include <stm32f10x.h>

// Helper functions
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
  for(int _l=1;_l<=16;_l++){
    uint16_t v = val & (1<<(16-_l));
    uart_send('0'+((v==0)?0:1));
  }
}
//

#define BUFF_LEN 32
static uint16_t timer1_capture_values_period[BUFF_LEN];
static uint16_t timer1_capture_values_high[BUFF_LEN];
static uint16_t timer2_capture_values_period[BUFF_LEN];
static uint16_t timer2_capture_values_high[BUFF_LEN];
static uint16_t timer3_capture_values_period[BUFF_LEN];
static uint16_t timer3_capture_values_high[BUFF_LEN];
static uint16_t timer4_capture_values_period[BUFF_LEN];
static uint16_t timer4_capture_values_high[BUFF_LEN];
static uint16_t timer_1_buf_idx;
static uint16_t timer_2_buf_idx;
static uint16_t timer_3_buf_idx;
static uint16_t timer_4_buf_idx;

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
  uint16_t *p_vp; // Pointer to store period from CH1
  uint16_t *h_vp; // Pointer to store high from CH2
  int16_t buf_idx; // index to current processed position in the buffer
  uint16_t *buf_update_idx_ptr; // index to current saved data in the buffer, set to -1 if not used
  int16_t x; // Derived x-angle measure from center
  int16_t y; // Derived y-angle measure from center
  uint16_t sync_pulse_code; // The decoded sync pulse value of current pulse
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
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = config.gpio_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(config.gpio_port, &GPIO_InitStructure);

  // Setup DMA only if used (buf_update_idx_ptr is set to 0 if interrupt not used)
  if (config.buf_update_idx_ptr==0){
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(config.dma_ch1);
    DMA_DeInit(config.dma_ch2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(config.timx->CCR1);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)config.p_vp;
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
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)config.h_vp;
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
  }

  if (config.timx_rcc == RCC_APB2Periph_TIM1){
    RCC_APB2PeriphClockCmd(config.timx_rcc, ENABLE);
    TIM_PrescalerConfig(config.timx, (72/config.resolution)-1, TIM_PSCReloadMode_Immediate); //71
  }
  else{
    RCC_APB1PeriphClockCmd(config.timx_rcc, ENABLE);
    TIM_PrescalerConfig(config.timx, (72/config.resolution)-1, TIM_PSCReloadMode_Immediate); //71
  }
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

  if (config.buf_update_idx_ptr==0){
    TIM_DMACmd(config.timx, TIM_DMA_CC1, ENABLE);
    TIM_DMACmd(config.timx, TIM_DMA_CC2, ENABLE);
  }

  TIM_Cmd(config.timx, ENABLE);

  // Setup ISR only if DMA not used
  if(config.buf_update_idx_ptr>0){
    TIM_ITConfig(config.timx, TIM_IT_CC1, ENABLE);
  }

  config._center *= config.resolution;
  config._sync_min *= config.resolution;
  config._sync_max *= config.resolution;
  config._sweep_max *= config.resolution;
}

#ifdef __cplusplus
 extern "C" {
#endif

void TIM1_CC_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
  timer1_capture_values_period[timer_1_buf_idx] = TIM_GetCapture1(TIM1);
  timer1_capture_values_high[timer_1_buf_idx] = TIM_GetCapture2(TIM1);
  timer_1_buf_idx++;
  if (timer_1_buf_idx>=BUFF_LEN){
    timer_1_buf_idx = 0;
  }
}

void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  timer2_capture_values_period[timer_2_buf_idx] = TIM_GetCapture1(TIM2);
  timer2_capture_values_high[timer_2_buf_idx] = TIM_GetCapture2(TIM2);
  timer_2_buf_idx++;
  if (timer_2_buf_idx>=BUFF_LEN){
    timer_2_buf_idx = 0;
  }
}

void TIM3_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
  timer3_capture_values_period[timer_3_buf_idx] = TIM_GetCapture1(TIM3);
  timer3_capture_values_high[timer_3_buf_idx] = TIM_GetCapture2(TIM3);
  timer_3_buf_idx++;
  if (timer_3_buf_idx>=BUFF_LEN){
    timer_3_buf_idx = 0;
  }
}

void TIM4_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
  timer4_capture_values_period[timer_4_buf_idx] = TIM_GetCapture1(TIM4);
  timer4_capture_values_high[timer_4_buf_idx] = TIM_GetCapture2(TIM4);
  timer_4_buf_idx++;
  if (timer_4_buf_idx>=BUFF_LEN){
    timer_4_buf_idx = 0;
  }
}

#ifdef __cplusplus
}
#endif

void process_sensor_data(TIM_PWM_CAPTURE_t &config)
{
  int16_t idx_h,idx_p;
  if(config.buf_update_idx_ptr>0){ // if interrupt used
    idx_p = *(config.buf_update_idx_ptr)-config.buf_idx;
    idx_h = *(config.buf_update_idx_ptr)-config.buf_idx;
  }
  else{ // if DMA used
    idx_p = BUFF_LEN-DMA_GetCurrDataCounter(config.dma_ch1)-config.buf_idx;
    idx_h = BUFF_LEN-DMA_GetCurrDataCounter(config.dma_ch2)-config.buf_idx;
  }
  idx_p = idx_p>=0?idx_p:BUFF_LEN+idx_p;
  idx_h = idx_h>=0?idx_h:BUFF_LEN+idx_h;
  if (idx_h>1 & idx_p>1){
    // status_led_activity();
    int16_t next_idx = config.buf_idx+1;
    if (next_idx==BUFF_LEN) next_idx=0;
    if ((config.h_vp)[config.buf_idx]<config._sync_max & (config.h_vp)[config.buf_idx]>config._sync_min) { // If valid sync pulse is found
      // 500 tics = 48uS
      // 1 uS = 500/48 tics
      // config.sync_pulse_code = int((0.096*tc_h[buf_idx])-5); // Decode the sync pulse
      config.sync_pulse_code = int((((48/config.resolution)*(config.h_vp)[config.buf_idx])-2750)/500); // Decode the sync pulse

      // Process position
      if ((config.h_vp)[next_idx]<config._sweep_max){ // If valid position is found after the sync pulse
        int16_t pos = config._center-(config.p_vp)[config.buf_idx]; // Relative angle from center (4000uS)
        if (pos<config._center & pos>-config._center){ // Double check if position is within range
          if (config.sync_pulse_code%2==0){ // This was a X sweep
            if (config.buf_update_idx_ptr>0){
              config.x = pos;
            }
            else{ //DMA generates reverse(x<->y) signal! Why?
              config.y = -pos;
            }
          }
          else{  // This was a Y sweep
            if (config.buf_update_idx_ptr>0){
              config.y = pos;
            }
            else{ //DMA generates reverse(x<->y) signal! Why?
              config.x = -pos;
            }
          }
        }
        config.buf_idx = next_idx+1; // Processed a sync and a sweep pulse
      }
      else{
        config.buf_idx = next_idx; // Processed a sync pulse only
      }
      if (config.buf_idx==BUFF_LEN) config.buf_idx=0;
    }
    else{
      // If not a valid sync pulse, skip to next
      config.buf_idx = next_idx;
    }
  }
}

#define OOTX_BUF_LEN 64
typedef enum OOTX_STATE{
  STOPPED=0,
  INITALIZED,
  WAIT_FOR_FRAME,
  CHECK_FRAME,
  PROCESS_FRAME
} OOTX_STATE_t;

typedef struct OOTX_DATA{
  OOTX_STATE_t state;
  uint8_t buf_index;
  uint8_t bit_counter;
  uint8_t preamble_received;
  uint32_t preamble;
  int8_t ax;
  int8_t ay;
  int8_t az;
  uint8_t valid;
  uint16_t buf[OOTX_BUF_LEN];
} OOTX_DATA_t;

void format_ootx_data(OOTX_DATA_t &ootx)
{
  ootx.ax = (int8_t)((ootx.buf[1+(20/2)]>>8)&0xff);
  ootx.ay = (int8_t)((ootx.buf[1+(21/2)])&0xff);
  ootx.az = (int8_t)((ootx.buf[1+(22/2)]>>8)&0xff);
  ootx.valid = 1;
}

void process_ootx_frame(OOTX_DATA_t &ootx, TIM_PWM_CAPTURE_t &data)
{
  if (data.sync_pulse_code==0xF) return; // Already processed
  // Process sync data
  uint16_t sync_pulse_code = (data.sync_pulse_code&0x0002)>>1;
  // Lookout for preamble
  // When found change updating buffer to avoid overwrite and change state
  ootx.preamble <<= 1;
  ootx.preamble |= sync_pulse_code;
  if ((ootx.preamble & 0x0003ffff) == 0x00000001){
    ootx.preamble_received = 1;
  }
  // State machine to distribute processing
  switch (ootx.state) {
    case STOPPED:
      // Do nothing, save time by not processing OOTX frame
      break;
    case INITALIZED:
      if(ootx.preamble_received){
        ootx.buf_index = 0;
        ootx.bit_counter = 0;
        ootx.preamble_received = 0;
        ootx.state = WAIT_FOR_FRAME;
      }
      break;
    case WAIT_FOR_FRAME:
      // Push bits into the ootx buffer
      if(ootx.preamble_received){
        ootx.preamble_received = 0;
        ootx.state = CHECK_FRAME;
      }
      else if(ootx.bit_counter<16){
        ootx.buf[ootx.buf_index] = (ootx.buf[ootx.buf_index]<<1)|sync_pulse_code;
        ootx.bit_counter++;
      }
      else{
        ootx.bit_counter = 0;
        ootx.buf_index++;
        // Error checking resets ootx frame finding process
        if(sync_pulse_code==0){ //Wrong sync bit
          if((ootx.preamble & 0x0001ffff)!=0){ // Wrong only if its not preamble being received
            ootx.state = INITALIZED;
            uart_sends("Error: Sync bit\r\n");
          }
        }
        if (ootx.buf_index==OOTX_BUF_LEN){ // Overflow
          ootx.state = INITALIZED;
          uart_sends("Error: Buffer overflow\r\n");
        }
      }
      break;
    case CHECK_FRAME:
      // Check payload length matchup
      // Check CRC
      ootx.state = PROCESS_FRAME;
      break;
    case PROCESS_FRAME:
      ootx.state = INITALIZED;
      format_ootx_data(ootx);
      break;
  }
  data.sync_pulse_code=0xF;
}

void transmit(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t x3,uint16_t y3,uint16_t x4,uint16_t y4)
{
  const char buf[] = {
    '>',
    (uint8_t) ((x1>>8) & 0xff),
    (uint8_t) (x1 & 0xff),
    (uint8_t) ((y1>>8) & 0xff),
    (uint8_t) (y1 & 0xff),
    (uint8_t) ((x2>>8) & 0xff),
    (uint8_t) (x2 & 0xff),
    (uint8_t) ((y2>>8) & 0xff),
    (uint8_t) (y2 & 0xff),
    (uint8_t) ((x3>>8) & 0xff),
    (uint8_t) (x3 & 0xff),
    (uint8_t) ((y3>>8) & 0xff),
    (uint8_t) (y3 & 0xff),
    (uint8_t) ((x4>>8) & 0xff),
    (uint8_t) (x4 & 0xff),
    (uint8_t) ((y4>>8) & 0xff),
    (uint8_t) (y4 & 0xff),
    '\r',
    '\n'
  };
  int buflen = sizeof(buf);
  int i=0;
  while(i<buflen){
    uart_send(buf[i++]);
  }
}

int main()
{
  char *formatted_str = (char*)malloc(128 * sizeof(char));
  uint16_t i=0;
  uint8_t _debug=1;
  uint8_t _serial_update=0;

  uart_init(115200,0);
  status_led_reset();
  delay(1000);

  TIM_PWM_CAPTURE_t configuration_sensor_1 = {
    .nvic_irqchannel = TIM1_CC_IRQn,
    .dma_ch1 = DMA1_Channel2,
    .dma_ch2 = DMA1_Channel3,
    .timx = TIM1,
    .timx_rcc = RCC_APB2Periph_TIM1,
    .gpio_rcc = RCC_APB2Periph_GPIOA,
    .gpio_pin = GPIO_Pin_8,
    .gpio_port = GPIOA,
    .resolution = 4,
    ._center = 4000,
    ._sync_min = 50,
    ._sync_max = 150,
    ._sweep_max = 40,
    .p_vp = timer1_capture_values_period,
    .h_vp = timer1_capture_values_high,
    .buf_idx = 0,
    .buf_update_idx_ptr = &timer_1_buf_idx
  };

  TIM_PWM_CAPTURE_t configuration_sensor_2 = {
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
    ._sweep_max = 40,
    .p_vp = timer4_capture_values_period,
    .h_vp = timer4_capture_values_high,
    .buf_idx = 0,
    .buf_update_idx_ptr = &timer_4_buf_idx
  };

  TIM_PWM_CAPTURE_t configuration_sensor_3 = {
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
    ._sweep_max = 40,
    .p_vp = timer2_capture_values_period,
    .h_vp = timer2_capture_values_high,
    .buf_idx = 0,
    .buf_update_idx_ptr = &timer_2_buf_idx
  };

  TIM_PWM_CAPTURE_t configuration_sensor_4 = {
    .nvic_irqchannel = TIM3_IRQn,
    .dma_ch1 = DMA1_Channel6,
    .dma_ch2 = 0, // No connection
    .timx = TIM3,
    .timx_rcc = RCC_APB1Periph_TIM3,
    .gpio_rcc = RCC_APB2Periph_GPIOA,
    .gpio_pin = GPIO_Pin_6,
    .gpio_port = GPIOA,
    .resolution = 4,
    ._center = 4000,
    ._sync_min = 50,
    ._sync_max = 150,
    ._sweep_max = 40,
    .p_vp = timer3_capture_values_period,
    .h_vp = timer3_capture_values_high,
    .buf_idx = 0,
    .buf_update_idx_ptr = &timer_3_buf_idx
  };

  OOTX_DATA_t ootx_data = {
    .state = INITALIZED,
    .buf_index = 0,
    .bit_counter = 0,
    .preamble_received = 0,
    .preamble = 0xffffffff
  };

  init_timer_capture(configuration_sensor_1);
  init_timer_capture(configuration_sensor_2);
  init_timer_capture(configuration_sensor_3);
  init_timer_capture(configuration_sensor_4);

  while(1){
    process_sensor_data(configuration_sensor_1);
    process_sensor_data(configuration_sensor_2);
    process_sensor_data(configuration_sensor_3);
    process_sensor_data(configuration_sensor_4);
    // process_ootx_frame(ootx_data,configuration_sensor_1);
    if(available()){
      if(uart_getc()=='#'){
        status_led_set();
        transmit(
          configuration_sensor_1.x,
          configuration_sensor_1.y,
          configuration_sensor_2.x,
          configuration_sensor_2.y,
          configuration_sensor_3.x,
          configuration_sensor_3.y,
          configuration_sensor_4.x,
          configuration_sensor_4.y
        );
        status_led_reset();
      }
    }
    else{
      // delay(1);
    }



    /*  _DEBUG stuff */
    if(_debug & ootx_data.valid){
      uart_send('#');
      uart_send_int(ootx_data.ax);
      uart_send(',');
      uart_send_int(ootx_data.ay);
      uart_send(',');
      uart_send_int(ootx_data.az);
      uart_sends("\r\n");
      ootx_data.valid = 0;
    }

    if (_debug & ootx_data.state==PROCESS_FRAME){
      for(int _l=0;_l<ootx_data.buf_index;_l++){
        // ootx_data.buf[_l] = ((ootx_data.buf[_l]<<8)&0xff00)|((ootx_data.buf[_l]>>8)&0x00ff);
        uart_send_uint(_l);
        uart_sends(":\t");
        uart_send_binary_complete(ootx_data.buf[_l]);
        uart_send('\t');
        uart_send_uint(ootx_data.buf[_l]);
        uart_sends("\r\n");
        // uart_send_uint(ootx_data.buf[_l]);
        // uart_send(',');
      }
      uint16_t tilt1 = ((ootx_data.buf[5+1]<<8)&0xff00)|((ootx_data.buf[5+1]>>8)&0x00ff);
      // uint16_t data_len = ((ootx_data.buf[0]<<8)&0xff00)|((ootx_data.buf[0]>>8)&0x00ff);
      // uint16_t protocol = (ootx_data.buf[1]>>8)&0x3f;
      // uint8_t ax = (ootx_data.buf[1+(20/2)]>>8)&0xff;
      // uint8_t ay = (ootx_data.buf[1+(21/2)])&0xff;
      // uint8_t az = (ootx_data.buf[1+(22/2)]>>8)&0xff;
      // int16_t _ax = (int8_t)((ootx_data.buf[1+(20/2)]>>8)&0xff);
      // int16_t _ay = (int8_t)((ootx_data.buf[1+(21/2)])&0xff);
      // int16_t _az = (int8_t)((ootx_data.buf[1+(22/2)]>>8)&0xff);
      // uart_send_int(_ax);
      // uart_send('\t');
      // uart_send_int(_ay);
      // uart_send('\t');
      // uart_send_int(_az);
      // uart_sends("\r\n\n\n");
    }
    if (_debug & i%50==0){
      uart_send_int(configuration_sensor_1.x);
      uart_send(',');
      uart_send_int(configuration_sensor_1.y);
      uart_send('\t');
      uart_send_int(configuration_sensor_2.x);
      uart_send(',');
      uart_send_int(configuration_sensor_2.y);
      uart_send('\t');
      uart_send_int(configuration_sensor_3.x);
      uart_send(',');
      uart_send_int(configuration_sensor_3.y);
      uart_send('\t');
      uart_send_int(configuration_sensor_4.x);
      uart_send(',');
      uart_send_int(configuration_sensor_4.y);
      // uart_send('\t');
      // uart_send_uint(configuration_sensor_2.p_vp[0]);
      // uart_send(',');
      // uart_send_uint(configuration_sensor_2.h_vp[0]);
      // uart_send('\t');
      // uart_send_uint(configuration_sensor_3.p_vp[0]);
      // uart_send(',');
      // uart_send_uint(configuration_sensor_3.h_vp[0]);
      uart_sends("\r\n");
    }
    if (_debug){
      delay(1);
    }
    i++;
  }
}
