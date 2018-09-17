#include <utils.h>

static volatile uint8_t status_led_initialized = 0;
static volatile uint8_t delay_initialized = 0;
static volatile uint32_t _delay = 0;
static volatile uint32_t _delay_resolution = 1;

void status_led_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  status_led_reset();
  status_led_initialized = 1;
}

void status_led_activity()
{
  if(!status_led_initialized) status_led_init();
  static uint8_t led_status = 0;
  GPIOC->BSRR = led_status?GPIO_BSRR_BS13:GPIO_BSRR_BR13;
  led_status = led_status?0:1;
}

void status_led_set()
{
  if(!status_led_initialized) status_led_init();
  GPIOC->BSRR = GPIO_BSRR_BR13;
}

void status_led_reset()
{
  if(!status_led_initialized) status_led_init();
  GPIOC->BSRR = GPIO_BSRR_BS13;
}

#ifdef __cplusplus
 extern "C" {
#endif
void SysTick_Handler(void)
{
  if(_delay>0) _delay--;
}
#ifdef __cplusplus
}
#endif

void delay_init(uint32_t resolution)
{
  _delay_resolution = constrain(resolution,1,1000);
  SysTick_Config(SystemCoreClock / (8000/_delay_resolution));
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
  delay_initialized = 1;
}

void delay(uint32_t millis)
{
  if(!delay_initialized) delay_init(10);
  _delay = millis/_delay_resolution;
  while(_delay>0);
}

void delay_nop(uint16_t millis)
{
  while (millis-- > 0) {
    volatile int x = 6000;
    while (x-- > 0) {
      __asm("nop");
    }
  }
}

void system_reset()
{
  NVIC_SystemReset();
}
