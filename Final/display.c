#include "fsl_device_registers.h"
#include <stdint.h>

// 7-segment patterns (common cathode) on PD0–PD6
int nums[10] = {
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011  // 9
};

#define RED_LED_MASK (1u << 7)   // PTD7

static inline void red_on(void)  { GPIOD_PSOR = RED_LED_MASK; }
static inline void red_off(void) { GPIOD_PCOR = RED_LED_MASK; }

// Display ones digit on PD0–PD6 (keep PD7 as LED)
void display_digit(uint8_t digit)
{
    if (digit > 9) digit = 9;
    uint32_t pattern = (uint32_t)(nums[digit] & 0x7F); // bits 0–6

    uint32_t current = GPIOD_PDOR & ~0x7F; // clear PD0–PD6, keep PD7
    GPIOD_PDOR = current | pattern;
}

int main(void)
{
    // Enable Port D clock
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // PD0–PD7 as GPIO
    PORTD_PCR0 = 0x100;
    PORTD_PCR1 = 0x100;
    PORTD_PCR2 = 0x100;
    PORTD_PCR3 = 0x100;
    PORTD_PCR4 = 0x100;
    PORTD_PCR5 = 0x100;
    PORTD_PCR6 = 0x100;
    PORTD_PCR7 = 0x100;

    // All PD0–PD7 outputs
    GPIOD_PDDR |= 0xFF;

    // Clear all
    GPIOD_PDOR &= ~0xFF;

    // Turn red LED on and show 9
    red_on();
    display_digit(9);

    while (1) {
        // do nothing; just keep outputs
    }
}
