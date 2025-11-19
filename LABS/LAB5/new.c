#include "fsl_device_registers.h"
#include <stdint.h>

#define STEP0  0x3Au   // Step 3 from slide (A2 & B2 + ENA/ENB)
#define STEP1  0x39u   // Step 2 from slide (A1 & B2 + ENA/ENB)
#define STEP2  0x35u   // Step 1 from slide (A1 & B1 + ENA/ENB)
#define STEP3  0x36u   // Step 0 from slide (A2 & B1 + ENA/ENB)

// simple blocking delay
static void delay(volatile uint32_t count)
{
    while (count--)
    {
        __asm("NOP");
    }
}

void main(void)
{
    // 1. Enable clock for PORTD
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // 2. Set PTD0–PTD5 MUX = 1 (GPIO)
    //    GPCLR: lower 16 bits = mask, upper 16 bits = value for PCRs
    //    0x003F = bits 0–5, 0x0100 sets MUX=1
    PORTD_GPCLR = 0x003F0100;

    // 3. Configure PTD0–PTD5 as outputs
    GPIOD_PDDR |= 0x0000003F;

    // 4. Start with all low
    GPIOD_PDOR = 0x00;

    while (1)
    {
        // CCW sequence (reverse of slide’s CW order)
        GPIOD_PDOR = STEP0;  // pattern 0x3A
        delay(800000);

        GPIOD_PDOR = STEP1;  // pattern 0x39
        delay(800000);

        GPIOD_PDOR = STEP2;  // pattern 0x35
        delay(800000);

        GPIOD_PDOR = STEP3;  // pattern 0x36
        delay(800000);
    }
}
