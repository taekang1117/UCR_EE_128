/* PORTB pin 2 is MODE_SW and pin 3 is CNTR Direction 
PORTC is connected to the 10's LED 
PORTD is connect to the 1's LED 
*/ 

#include "fsl_device_registers.h"
#include <stdint.h>

unsigned short ADC_read16b(void); // For ADC_read
// TO DO: Initialization 
void main(void) {
    // 1. CLOCK GATING
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Enable PORTA, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable PORTB, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Enable PORTC, Clock Gating
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable PORTD, Clock Gating
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // Enable ADC0 Clock
    
    // 2. GPIO SETUP
    PORTA_PCR1 = 0xA0100; // Configure Port A pin 1 for GPIO Interrupt on falling edge    
    PORTB_GPCLR = 0x000C0100; // GPIO Setup: PORTB pin 2 and 3
    PORTC_GPCLR = 0x01BF0100 // GPIO Setup: LED1 8 pins 0,1,2,3,4,5,7
    PORTD_GPCLR = 0x00FF0100 // GPIO Setup: LED2 8 pins 
    ADC0_CFG1 = 0x0C; // 16 bits ADC, Bus Clock
    ADC0_SC1A = 0x1F; // Disable the module during init, ADCH = 11111

    // 3. GPIO DIRECTION
    GPIOB_PDDR = 0x00000000// DIP Switch 
    GPIOC_PDDR = 0x000001BF// LED1
    GPIOD_PDDR = 0x000000FF// LED2
    

    PORTA_ISFR = (1<<1); // Clear ISFR for Port A pin 1
    NVIC_EnableIRQ(PORTA_IRQn); // Enable Port A interrupt in

    // 4. High Low
    GPIOB_PDOR = 0x0000000C; // Set Pins 2 and 3 on Port B to be high voltage
    GPIOC_PDOR = 0x00000000; // Set Pins 0-5, 7-8 on Port C to be low voltage
    GPIOD_PDOR = 0x000000FF; // Set Pins 0-7 on Port D to be low voltage
    
 // TO DO: Main code loop
    while(1) {
        
}

// TO DO: void PORTA_IRQHandler(void) 
unsigned 
// TO DO: Interrupt Code

