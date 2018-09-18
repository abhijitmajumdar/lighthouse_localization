#include "comm.h"
#include "utils.h"

int main()
{
  char *formatted_str = (char*)malloc(128 * sizeof(char));
  uint16_t i=0;
  uart_init(115200,0);
  while(1){
    i++;
    uart_clear_line(64);
    sprintf(formatted_str,"stm32 > %d\r",i);
    uart_sends(formatted_str);
    if(available()){
      while(available()){
        switch(uart_getc()){
          case 'r':
            uart_sends("Resetting...");
            uart_deinit();
            system_reset();
            break;
          case 's':
            uart_sends("\nReinit delay\n");
            delay_init(200);
            break;
          default:
            uart_sends("\r\n!!!!!!!!......INVALID INPUT......!!!!!!!!\r\n");
        }
      }
    }
    status_led_activity();
    delay(500);
  }
}
