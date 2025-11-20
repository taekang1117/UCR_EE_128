#include "fsl_device_registers.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// Port D bit masks (motor control)
// PD0: A1, PD1: A2, PD2: B1, PD3: B2, PD4: ENA, PD5: ENB
// -----------------------------------------------------------------------------
#define A1_BIT   (1u << 0)
#define A2_BIT   (1u << 1)
#define B1_BIT   (1u << 2)
#define B2_BIT   (1u << 3)
#define ENA_BIT  (1u << 4)
#define ENB_BIT  (1u << 5)

// Two-phase full-step sequence (same as before)
#define STEP0  0x36u   // ENA, ENB, A2, B1 high
#define STEP1  0x35u   // ENA, ENB, A1, B1 high
#define STEP2  0x39u   // ENA, ENB, A1, B2 high
#define STEP3  0x3Au   // ENA, ENB, A2, B2 high

// Calibrated delays (from your measurements)
#define DELAY_22_5   195000u   // ~22.5 deg/s
#define DELAY_180     25000u   // ~180 deg/s

// DIP switch pins: PTB2, PTB3
#define SW2_BIT  (1u << 2)    // PTB2
#define SW3_BIT  (1u << 3)    // PTB3

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
// Initialize Port B: PTB2, PTB3 as GPIO inputs (DIP switches) with pull-ups
// -----------------------------------------------------------------------------
static void init_portB_switches(void)
{
    // Enable clock to Port B
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // PTB2, PTB3 as GPIO (MUX = 1), enable internal pull-ups
    PORTB_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    // Set PTB2, PTB3 as inputs (clear bits in PDDR)
    GPIOB_PDDR &= ~(SW2_BIT | SW3_BIT);
}

// -----------------------------------------------------------------------------
// Do one 4-step cycle CCW with given delay
// -----------------------------------------------------------------------------
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
    init_portD();
    init_portB_switches();

    while (1)
    {
        // Read current state of PTB2 and PTB3
        uint32_t portb = GPIOB_PDIR;
        uint8_t sw2 = (portb & SW2_BIT) ? 1u : 0u;  // PTB2
        uint8_t sw3 = (portb & SW3_BIT) ? 1u : 0u;  // PTB3

        uint32_t delay_val;

        // Truth table:
        // sw2 sw3
        //  0   0  -> CW 22.5
        //  0   1  -> CW 180
        //  1   0  -> CCW 22.5
        //  1   1  -> CCW 180

        if (sw2 == 0 && sw3 == 0)
        {
            delay_val = DELAY_22_5;
            step_cw(delay_val);
        }
        else if (sw2 == 0 && sw3 == 1)
        {
            delay_val = DELAY_180;
            step_cw(delay_val);
        }
        else if (sw2 == 1 && sw3 == 0)
        {
            delay_val = DELAY_22_5;
            step_ccw(delay_val);
        }
        else // sw2 == 1 && sw3 == 1
        {
            delay_val = DELAY_180;
            step_ccw(delay_val);
        }

        // Because we re-read the switches every 4 steps,
        // changing the DIP while running will change speed/direction "on the fly".
    }

    // Never reached
    return 0;
}
