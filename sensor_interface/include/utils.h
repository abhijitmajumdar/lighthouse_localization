#ifndef _UTILS_H
#define _UTILS_H

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

// Helper macros
#define constrain(v,min,max) v>=min?(v<=max?v:max):min

// Status led on PC13
void status_led_init();
void status_led_activity();
void status_led_set();
void status_led_reset();

// delay_systick() uses SysTick timer, set to a resolution of 100ms(to have less footprint)
// Resolution can be reset using delay_init() in range [1,1000]ms
// Note: If using delay less than resolution,there is no delay
// delay_nop() uses loops over NOP instructions
// The deafult delay type can be defined as either -DDELAY_TYPE_NOP or -DDELAY_TYPE_SYSTICK
// in the makefile as a comiler flag, either of which can be called using delay()

void delay_init(uint32_t resolution);
void delay_systick(uint16_t millis);
void delay_nop(uint16_t millis);

#ifdef DELAY_TYPE_NOP
  #define delay delay_nop
#endif
#ifdef DELAY_TYPE_SYSTICK
  #define delay delay_systick
#endif

// NVIC system reset
void system_reset();

#ifdef __cplusplus
}
#endif

#endif //_UTILS_H
