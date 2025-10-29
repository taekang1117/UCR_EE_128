#include "fsl_device_registers.h"
#include <stdint.h>

volatile unsigned int cnt = 0;

void software_delay(unsigned long delay) {
    while (delay > 0) delay--;
}

int numsPTD[10] = {
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011, // 9
    }; 

int numsPTC[10] = {
    0b010111110, // 0 done
    0b000110000, // 1 done
    0b010101101, // 2 done
    0b010111001, // 3 done
    0b000110011, // 4 done
    0b100011011, // 5
    0b000011111, // 6
    0b100110000, // 7
    0b000111111, // 8
    0b000110011, // 9
    }; 
  
  //TODO: complete outNum()
void outNumPTC(int num){
    // GPIOC_PDOR = numsPTC[num];
    GPIOC_PDOR = (~numsPTC[num]) & 0x000000BF; // Output register
}

void outNumPTD(int num) {
    GPIOD_PDOR = (~numsPTD[num]) & 0x000003BF;
}


// unsigned short ADC_read16b(void) {
//     ADC0_SC1A = 0x1A;
//     while (ADC0_SC2 & ADC_SC2_ADACT_MASK);
//     while (!(ADC0_SC1A & ADC_SC1_COCO_MASK));
//     return ADC0_RA;
// }

unsigned short ADC_read16b(void){
    ADC0_SC1A = 0x00;                              // SE channel 0
    while (ADC0_SC2 & ADC_SC2_ADACT_MASK) {}       // wait active
    while (!(ADC0_SC1A & ADC_SC1_COCO_MASK)) {}    // wait done
    return ADC0_RA;
}

void PORTA_IRQHandler(void) {
    // Toggle ones place decimal point;
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    GPIOD_PTOR |= 0xFF;

    // Read Port B
    const uint32_t MODE_SW = 0x4;
    const uint32_t CNT_DIR = 0x8;
    
    if (GPIOB_PDIR & MODE_SW) { /* ADC Mode */
        // ADC_read()
        unsigned short voltageNum = ADC_read16b();
        unsigned short firstDigit = voltageNum % 10;
        unsigned short secondDigit = (voltageNum - firstDigit) / 10;
        outNumPTC(firstDigit);
        outNumPTD(secondDigit);
    }
    // Read from ADC and convert to decimal value; (e.g., ADC reads 0xFF, voltage is 1.6)
    if(!(GPIOB_PDIR & MODE_SW)) {  /* Count Mode */
        if (!(GPIOB_PDIR & CNT_DIR)) { /* Count Direction */
            if (cnt < 99) {
                cnt ++;
            }
            else {
                cnt = 0;
            }
        }
        else if (GPIOB_PDIR & CNT_DIR) {
            if (cnt > 0) {
                cnt--;
            }
            else {
                cnt = 99;
            }
        }
        unsigned short firstDigit = cnt % 10;
        unsigned short secondDigit = (cnt - firstDigit) / 10;
        outNumPTC(firstDigit);
        outNumPTD(secondDigit);
    }

    //Display ADC value or Counter value based on MODE_SW; /* PORT C and D to seven segments */
    // 29 % 10 = 9. PTC
    // (29 - (29 % 10)) / 10 = 2. PTD
    //Clear ISFR
    PORTA_ISFR = (1 << 1);
}


void main(void) {
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;  /* Enable Port A Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  /* Enable Port B Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  /* Enable Port C Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;  /* Enable Port D Clock Gate Control*/


    // PORTA_GPCLR = 0x000A0100;   /* Configures Pins 1 and 3 on Port A to be GPIO */
    // PORTB_GPCLR = 0x000C0100;   /* Configures Pins 2 and 3 on Port B to be GPIO */
    // PORTC_GPCLR = 0x005F0100;   /* Configures Pins 0-5, 7 on Port C to be GPIO */
    // PORTD_GPCLR = 0x007F0100;   /* Configures Pins 0-6 on Port D to be GPIO */

    PORTA_GPCLR = (0x0100 << 16) | 0x0002;   // PTA1 as GPIO + (later set IRQC in PCR)
    PORTB_GPCLR = (0x0100 << 16) | 0x000C;   // PTB2-3 GPIO
    PORTC_GPCLR = 0x01BF0100;   // PTC0-5,7 GPIO
    PORTD_GPCLR = 0x007F0100;   // PTD0-7 GPIO  (need DP bit too!)

    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;


    GPIOA_PDDR = 0x00000000;    /* Configures Pins 1 and 3 of port A as Input */
    GPIOB_PDDR = 0x00000000;    /* Configures Pins 2 and 3 of port B as Input */
    GPIOC_PDDR = 0x000001BF;    /* Configures Pins 0-5, 7 on Port C as Output */
    GPIOD_PDDR = 0x000000FF;    /* Configures Pins 0-6 on Port D as Output */

    GPIOA_PDOR = 0x0000000A;    /* Set Pins 1 and 3 on Port A to be high voltage */
    GPIOB_PDOR = 0x0000000C;    /* Set Pins 2 and 3 on Port B to be high voltage */
    GPIOC_PDOR &= ~(0x000001BF);		//set to high
    GPIOD_PDOR &= ~(0x0000007F);		//set to high


    unsigned long Delay = 0x0010;

    ADC0_CFG1 = 0x0C;
    ADC0_SC1A = 0x1F;
    
    PORTA_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xA); // falling-edge
    PORTA_ISFR = (1<<1);
    while (1) {
        software_delay(Delay);
    }
}



