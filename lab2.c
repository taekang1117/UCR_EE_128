#include "fsl_device_registers.h"
#include <stdint.h>

// TO DO: Initialization 
void main(void) {
    // 1. CLOCK GATING
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Enable PORTA, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable PORTB, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Enable PORTC, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable PORTD, Clock Gating

    // 2. GPIO SETUP
    PORTA_PCR1 = 0xA0100; // Configure Port A pin 1 for GPIO Interrupt on falling edge    
    PORTB_GPCLR = 0x000C0100; // GPIO Setup: PORTB pin 2 and 3
    PORTC_GPCLR = // GPIO Setup: LED1 8 pins 
    PORTD_GPCLR = // GPIO Setup: LED2 8 pins 

    // 3. GPIO DIRECTION
    GPIOB_PDDR = // DIP Switch 
    GPIOC_PDDR = // LED1
    GPIOD_PDDR = // LED2


    PORTA_ISFR = (1<<1); // Clear ISFR for Port A pin 1
    NVIC_EnableIRQ(PORTA_IRQn); // Enable Port A interrupt in

    unit32_t inval = 0;
    
 // TO DO: Main code loop

}

// TO DO: void PORTA_IRQHandler(void) 
// TO DO: Interrupt Code

