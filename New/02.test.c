#include "fsl_device_registers.h"
#include <stdint.h>

#define SYSTEM_CLOCK 48000000u

// same patterns you used on UNO
int nums[16] =
{
  0b0111111, // 0
  0b0000110, // 1
  0b1011011, // 2
  0b1001111, // 3
  0b1100110, // 4
  0b1101101, // 5
  0b1111101, // 6
  0b0000111, // 7
  0b1111111, // 8
  0b1101111, // 9
  0b1110111, // a
  0b1111100, // b
  0b0111001, // c
  0b1011110, // d
  0b1111001, // e
  0b1110001  // f
};

#define SEG_MASK    0x7F      // PTD0–PTD6
#define DIGIT_MASK  (1u << 7) // PTD7

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++) {
        for(volatile uint32_t j = 0; j < (SYSTEM_CLOCK/10000u); j++) {
            __NOP();
        }
    }
}

void outNum(uint8_t num)
{
    if (num > 15) num = 0;

    uint32_t pattern = (uint32_t)(nums[num] & 0x7F);

    // write segments a–g to PTD0–PTD6 (active HIGH)
    uint32_t d = GPIOD_PDOR;
    d &= ~SEG_MASK;
    d |= pattern;          // bit0->PD0 ... bit6->PD6
    GPIOD_PDOR = d;
}

int main(void)
{
    // enable PORTD clock
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // PTD0–PTD7 as GPIO
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_MUX(1);
    PORTD_PCR6 = PORT_PCR_MUX(1);
    PORTD_PCR7 = PORT_PCR_MUX(1);

    // outputs
    GPIOD_PDDR |= (SEG_MASK | DIGIT_MASK);

    // start with digit OFF (assuming common-cathode, active LOW)
    GPIOD_PSOR = DIGIT_MASK; // PTD7 = 1 -> digit off

    while (1) {
        // enable the digit: common cathode = LOW
        GPIOD_PCOR = DIGIT_MASK;  // PTD7 = 0, digit ON

        for (uint8_t n = 0; n < 10; n++) {
            outNum(n);
            delay_ms(500);
        }

        // turn digit off for a bit
        GPIOD_PSOR = DIGIT_MASK;  // PTD7 = 1
        delay_ms(1000);
    }
}
