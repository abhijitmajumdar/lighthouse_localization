#include <iostream>
#include <unistd.h>
#include <string.h>
#include "gpio_access.h"

struct OPTIONS{
  bool bootloader;
  bool no_reset;
  int reset_pin;
  int boot_pin;
};

void parse_args(int ac, char** av, OPTIONS &op)
{
  op.bootloader = false;
  op.no_reset = false;
  op.boot_pin = -1;
  op.reset_pin = -1;
  for (int idx=0; idx<ac; idx++){
    if(strcmp(av[idx],"bootload")==0){
      op.bootloader = true;
    }
    else if(strcmp(av[idx],"noreset")==0){
      op.no_reset = true;
    }
    else if(strcmp(av[idx],"bpin")==0){
      idx++;
      if (idx<ac) op.boot_pin = std::atoi(av[idx]);
    }
    else if(strcmp(av[idx],"rpin")==0){
      idx++;
      if (idx<ac) op.reset_pin = std::atoi(av[idx]);
    }
  }
  if (op.boot_pin<0) op.boot_pin = 363;
  if (op.reset_pin<0) op.reset_pin = 203;
}

int main(int argc, char **argv)
{
  OPTIONS options;
  GPIO* bpin;
  GPIO* rpin;
  parse_args(argc,argv,options);

  // Check boot option
  bpin = new GPIO(options.boot_pin, OUT);
  if(options.bootloader){
    std::cout<<"Enabling bootloading\n";
    bpin->set(true);
  }
  else{
    std::cout<<"Disabling bootloading\n";
    bpin->set(false);
  }
  // Some dealy
  usleep(1000000);
  // The pin state is maintained even when the reference object is deleted
  delete bpin;

  // Device resets as soon as pin initialized, since it sinks reset pin
  if(!options.no_reset){
    rpin = new GPIO(options.reset_pin, OUT);
    rpin->set(false);
    std::cout<<"Resetting device\n";
    // Some dealy
    usleep(1000000);
    // Remove reset
    rpin->set(true);
    // Some more delay
    usleep(2000000);
    delete rpin;
  }

  return 0;
}
