#include "fsl_device_registers.h"
#include <stdint.h>

void main (void) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable Port B Clock Gate Control
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable port D Clock Gate Control
  PORTB_GPCLR = 0x000C0100; // Configures Pins 2 and 3 on Port B to be GPIO 
  // Global Pin Control Low Register [15:0] 
  // 000c = 0000 0000 0000 1100 
  // 0100 = GPIO 
  PORTD_GPCLR = 0x00FF0100; // Configures Pins 0-7 on Port D to be GPIO
  GPIOB_PDDR = 0x00000000; // Configures Pins 2 and 3 of Port B as Input 
  // PDDR means Port Data Direction Register, 1 = output, 0 = input 
  GPIOD_PDDR = 0x000000FF; // Configures Pins 0 - 7 on Port D to be output

  unit32_t_outval = 0, inval = 0; 
  while(1) {
    outval = 0;
    inval = GPIOB_PDIR & 0x0C; // Port Data Input Register. Used to READ input
    if(inval & 0x8) outval = 0x0F;
    if(inval & 0x4) outval |= 0xF0; // Lights up the lower and or upper half of LED bar

    GPIOD_PDOR = outval; // Port Data Output Register. 
  }
}
