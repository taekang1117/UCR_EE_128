#include "fsl_device_registers.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// Port D bit masks (motor control)
// PD0: A1, PD1: A2, PD2: B1, PD3: B2, PD4: ENA, PD5: ENB
// -----------------------------------------------------------------------------
#define A1_BIT   (1 << 0)
#define A2_BIT   (1 << 1)
#define B1_BIT   (1 << 2)
#define B2_BIT   (1 << 3)
#define ENA_BIT  (1 << 4)
#define ENB_BIT  (1 << 5)


#define STEP0  0x36   // ENA, ENB, A2, B1 high
#define STEP1  0x35   // ENA, ENB, A1, B1 high
#define STEP2  0x39   // ENA, ENB, A1, B2 high
#define STEP3  0x3A   // ENA, ENB, A2, B2 high


#define DELAY_22_5   195000   // ~22.5 deg/s
#define DELAY_180     25000   // ~180 deg/s

// DIP switch pins: PTB2, PTB3
#define SW2_BIT  (1 << 2)    // PTB2
#define SW3_BIT  (1 << 3)    // PTB3

static void delay_loop(volatile unsigned long count)
{
    while (count--)
    {
        __asm("NOP");
    }
}

static void step_ccw(uint32_t delay)
{
    // CCW: STEP0 -> STEP1 -> STEP2 -> STEP3
    GPIOD_PDOR = STEP0;
    delay_loop(delay);

    GPIOD_PDOR = STEP1;
    delay_loop(delay);

    GPIOD_PDOR = STEP2;
    delay_loop(delay);

    GPIOD_PDOR = STEP3;
    delay_loop(delay);
}

// -----------------------------------------------------------------------------
// Do one 4-step cycle CW with given delay
// -----------------------------------------------------------------------------
static void step_cw(uint32_t delay)
{
    // CW is just the reverse order: STEP3 -> STEP2 -> STEP1 -> STEP0
    GPIOD_PDOR = STEP3;
    delay_loop(delay);

    GPIOD_PDOR = STEP2;
    delay_loop(delay);

    GPIOD_PDOR = STEP1;
    delay_loop(delay);

    GPIOD_PDOR = STEP0;
    delay_loop(delay);
}

// -----------------------------------------------------------------------------
// Main program: poll DIP switches, choose direction + speed, and step motor
// -----------------------------------------------------------------------------
int main(void)
{
	// Enable clock to Port D and B
	    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
		SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	    // Set PTD0–PTD5 multiplexing to GPIO (MUX = 1)
	    PORTD_PCR0 = PORT_PCR_MUX(1);
	    PORTD_PCR1 = PORT_PCR_MUX(1);
	    PORTD_PCR2 = PORT_PCR_MUX(1);
	    PORTD_PCR3 = PORT_PCR_MUX(1);
	    PORTD_PCR4 = PORT_PCR_MUX(1);
	    PORTD_PCR5 = PORT_PCR_MUX(1);

	    // Configure PTD0–PTD5 and PTB2-3
	    GPIOD_PDDR |= (A1_BIT | A2_BIT | B1_BIT | B2_BIT | ENA_BIT | ENB_BIT);
	    PORTB_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	    PORTB_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;


	    // Start with all outputs low
	    GPIOD_PDOR = 0x00;

		// Set PTB2, PTB3 as inputs (clear bits in PDDR)
		GPIOB_PDDR &= ~(SW2_BIT | SW3_BIT);

    while (1)
    {
        // Read current state of PTB2 and PTB3
        uint32_t portb = GPIOB_PDIR;
        uint8_t sw2 = (portb & SW2_BIT) ? 1 : 0;  // PTB2
        uint8_t sw3 = (portb & SW3_BIT) ? 1 : 0;  // PTB3

        uint32_t delay_val;

        if (sw2 == 1 && sw3 == 1)
        {
            delay_val = DELAY_22_5;
            step_cw(delay_val);
        }
        else if (sw2 == 1 && sw3 == 0)
        {
            delay_val = DELAY_180;
            step_cw(delay_val);
        }
        else if (sw2 == 0 && sw3 == 1)
        {
            delay_val = DELAY_22_5;
            step_ccw(delay_val);
        }
        else
        {
            delay_val = DELAY_180;
            step_ccw(delay_val);
        }
    }

    return 0;
}
