#include "comm.h"
#include "utils.h"

void init_exti()
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    activity();

    EXTI_ClearITPendingBit(EXTI_Line11);
  }
}
*/

#define BUFF_LEN 4
static uint16_t timer_capture_values_period[BUFF_LEN];
static uint16_t timer_capture_values_high[BUFF_LEN];
void init_timer_capture()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Channel5);
  DMA_DeInit(DMA1_Channel7);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR1);
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
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR2);
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
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel5, ENABLE);
  DMA_Cmd(DMA1_Channel7, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_PrescalerConfig(TIM2, 71, TIM_PSCReloadMode_Immediate);
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  //TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
  TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);

  TIM_Cmd(TIM2, ENABLE);

  // TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  // TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}

// static uint16_t ph = 0;
// static uint16_t pl = 0;
// static uint16_t pt = 0;
/*
void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  uint16_t val = TIM_GetCapture1(TIM2);
  if (val){
    ph = TIM_GetCapture2(TIM2);
    pl = val - ph;
  }
  activity();
}
*/
/*
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET){
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    ph = TIM_GetCapture2(TIM2);
  }
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET){
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    pt = TIM_GetCapture1(TIM2);
  }
  activity();
}
*/

void init_dummy_pwm(uint32_t frequency)
{
  uint16_t overflow_value = 1000000/frequency;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  TIM_TimeBaseStructure.TIM_Period = overflow_value;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = overflow_value/2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

/*
static int some_global_indexer = 0;
static int buffer[4];
void INPUT_CAPTURE_BOTH_EDGE_CH_ISR(void)
{
  val = capture_value();
  if (60 uS < val < 140 uS){
    some_global_indexer = 0;
  }
  if (some_global_indexer>3){
    no_signal();
    return;
  }
  buffer[some_global_indexer++] = val;
}
*/

int main()
{
  char *formatted_str = (char*)malloc(128 * sizeof(char));
  uint16_t i=0;
  uint32_t freq = 120;
  uint16_t period = 8333;
  uint16_t duty = 62;
  uint16_t jump = 1;
  uint16_t *tc_h = timer_capture_values_high;
  uint16_t *tc_p = timer_capture_values_period;
  float x,y;
  uart_init(115200,0);
  // init_exti();
  // init_dummy_pwm(freq);
  init_timer_capture();
  while(1){
    i++;
    // sprintf(formatted_str, "----<%d::d=%d,p=%d,j=%d:\t%d,%d,%d>----\r",i,duty,period,jump,ph,pl,pt);
    // uint16_t h1,l1,h2,l2,h3,l3,h4,l4;
    // h1 = tc_h[0]+1;
    // l1 = tc_p[0]-h1;
    // h2 = tc_h[1]+1;
    // l2 = tc_p[1]-h2;
    // h3 = tc_h[2]+1;
    // l3 = tc_p[2]-h1;
    // h4 = tc_h[3]+1;
    // l4 = tc_p[3]-h2;
    // sprintf(formatted_str, "----<%d::d=%d,p=%d,j=%d:\t__/\t%d\t\\__%d__/\t%d\t\\__%d>----\r",i,duty,period,jump,h1,l1,h2,l2);
    // sprintf(formatted_str, "----<%d::d=%d,p=%d,j=%d:__/%d\\__%d__/%d\\__%d__/%d\\__%d__/%d\\__%d>----\r",i,duty,period,jump,h1,l1,h2,l2,h3,l3,h4,l4);

    for (uint8_t idx=0;idx<BUFF_LEN-1;idx++) { // For all records in buffer
      if (tc_h[idx]<150 & tc_h[idx]>50) { // If valid sync pulse is found
        uint16_t sync_pulse_code = int((0.096*tc_h[idx])-5); // Decode the sync pulse
        if (tc_h[idx+1]<40){ // If valid position is found after the sync pulse
          float pos = (4000-tc_p[idx])/90.0; // Relative angle from center (4000uS)
          if (pos<90 & pos>-90){ // Double check if position is within range
            if (sync_pulse_code%2==0){ // This was a X sweep
              x = pos;
            }
            else{  // This was a Y sweep
              y = pos;
            }
          }
          idx++;
        }
      }
      sprintf(formatted_str, "----<%d::x=%f,y=%f>----\r",i,x,y);
      // uart_clear_line(64);
      uart_sends(formatted_str);
      delay(50);
    }
    if(available()){
      while(available()){
        switch(uart_getc()){
          case 'r':
            uart_sends("Resetting...");
            uart_deinit();
            NVIC_SystemReset();
            break;
          case 'w':
            duty += jump;
            TIM_SetCompare1(TIM3,duty);
            break;
          case 's':
            duty -= jump;
            TIM_SetCompare1(TIM3,duty);
            break;
          case 'i':
            period += jump;
            TIM_SetAutoreload(TIM3,period);
            break;
          case 'j':
            period -= jump;
            TIM_SetAutoreload(TIM3,period);
            break;
          case 'd':
            jump += 1;
            break;
          case 'a':
            jump -= 1;
            break;
          default:
            uart_sends("\r\n!!!!!!!!......INVALID INPUT......!!!!!!!!\r\n");
        }
      }
    }
    // delay(100);
  }
}
