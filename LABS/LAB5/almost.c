#include "fsl_device_registers.h"

// -----------------------------------------------------------------------------
// Port D bit masks
// PD0: A1, PD1: A2, PD2: B1, PD3: B2, PD4: ENA, PD5: ENB
// -----------------------------------------------------------------------------
#define A1_BIT   (1u << 0)
#define A2_BIT   (1u << 1)
#define B1_BIT   (1u << 2)
#define B2_BIT   (1u << 3)
#define ENA_BIT  (1u << 4)
#define ENB_BIT  (1u << 5)

// From the slide (two-phase full-step sequence):
// 0x36 = 0011 0110 : ENA, ENB, A2, B1 high
// 0x35 = 0011 0101 : ENA, ENB, A1, B1 high
// 0x39 = 0011 1001 : ENA, ENB, A1, B2 high
// 0x3A = 0011 1010 : ENA, ENB, A2, B2 high
#define STEP0  0x36u
#define STEP1  0x35u
#define STEP2  0x39u
#define STEP3  0x3Au

// Tune this value so the average speed is about 22.5 deg/s.
// Start big so you can clearly see the motion, then decrease.
#define DELAY_COUNT1  195000u
#define DELAY_COUNT2  25000u

uint32_t inval = 0;
int speed = 0;
// -----------------------------------------------------------------------------
// Simple delay loop
// -----------------------------------------------------------------------------
static void delay_loop(volatile unsigned long count)
{
    while (count--)
    {
        __asm("NOP");
    }
}

// -----------------------------------------------------------------------------
// Initialize Port D: pins 0–5 as GPIO outputs, all low at start
// -----------------------------------------------------------------------------
static void init_portD(void)
{
    // Enable clock to Port D
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable Port B Clock Gate Control

    PORTB_GPCLR = 0x000C0100;
    GPIOB_PDDR = 0x00000000;

    // Set PTD0–PTD5 multiplexing to GPIO (MUX = 1)
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_MUX(1);

    // Configure PTD0–PTD5 as outputs
    GPIOD_PDDR |= (A1_BIT | A2_BIT | B1_BIT | B2_BIT | ENA_BIT | ENB_BIT);

    // Start with all outputs low
    GPIOD_PDOR = 0x00u;
}

// -----------------------------------------------------------------------------
// Main program: repeat 4-step sequence from the slide
// -----------------------------------------------------------------------------
int main(void)
{
    init_portD();
    speed = DELAY_COUNT1;

    while (1)
    {
		inval = GPIOB_PDIR; // Port Data Input Register. Used to READ input on Port B

		if((inval >> 3 & 0x1) == 1)
		{
			// Step 0: A2 and B1 high, ENA/ENB enabled
			speed = DELAY_COUNT2;
		}
		else
		{
			// Step 0: A2 and B1 high, ENA/ENB enabled
			speed = DELAY_COUNT1;
		}

		if((inval >> 3 & 0x1) == 1)
		{
			// Step 0: A2 and B1 high, ENA/ENB enabled
			GPIOD_PDOR = STEP0;
			delay_loop(speed);

			// Step 1: A1 and B1 high
			GPIOD_PDOR = STEP1;
			delay_loop(speed);

			// Step 2: A1 and B2 high
			GPIOD_PDOR = STEP2;
			delay_loop(speed);

			// Step 3: A2 and B2 high
			GPIOD_PDOR = STEP3;
			delay_loop(speed);
		}
		else
		{
			// Step 0: A2 and B1 high, ENA/ENB enabled
			GPIOD_PDOR = STEP3;
			delay_loop(speed);

			// Step 1: A1 and B1 high
			GPIOD_PDOR = STEP2;
			delay_loop(speed);

			// Step 2: A1 and B2 high
			GPIOD_PDOR = STEP1;
			delay_loop(speed);

			// Step 3: A2 and B2 high
			GPIOD_PDOR = STEP0;
			delay_loop(speed);
		}

    }

    // Never reached
    return 0;
}
