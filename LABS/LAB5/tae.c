#include "fsl_device_registers.h"

// -----------------------------------------------------------------------------
// PIN DEFINITIONS (PORT D)
// -----------------------------------------------------------------------------

// Motor Phase A (Connected to L298 IN1 & IN2 -> OUT1/OUT2 -> A1/A2)
#define A1_PIN   (1u << 0)   // PTD0
#define A2_PIN   (1u << 1)   // PTD1

// Motor Phase B (Connected to L298 IN3 & IN4 -> OUT3/OUT4 -> B1/B2)
#define B1_PIN   (1u << 2)   // PTD2
#define B2_PIN   (1u << 3)   // PTD3

// L298 Enable Pins
#define ENA_PIN  (1u << 4)   // PTD4 (Enables OUT1/OUT2)
#define ENB_PIN  (1u << 5)   // PTD5 (Enables OUT3/OUT4)

// Stepper control sequence (one phase on at a time, CW)
const unsigned char step_sequence[4] = {
    (A1_PIN | ENA_PIN | ENB_PIN),   // Step 1: A1 on
    (B1_PIN | ENA_PIN | ENB_PIN),   // Step 2: B1 on
    (A2_PIN | ENA_PIN | ENB_PIN),   // Step 3: A2 on
    (B2_PIN | ENA_PIN | ENB_PIN)    // Step 4: B2 on
};

// Adjust this to get ~22.5 deg/s depending on your CPU clock and motor step angle
#define STEP_DELAY_CYCLES  2500000UL

// -----------------------------------------------------------------------------
// FUNCTION PROTOTYPES
// -----------------------------------------------------------------------------
void init_portD_gpio(void);
void software_delay(unsigned long cycles);
void rotate_stepper_clockwise(void);
void set_step_output(unsigned char output_pattern);

// -----------------------------------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------------------------------

void init_portD_gpio(void) {
    // 1. Enable Clock for Port D
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // 2. Configure PTD0–PTD5 as GPIO (MUX = 1)
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_MUX(1);

    // 3. Set PTD0–PTD5 as outputs
    GPIOD_PDDR |= (A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);

    // 4. Initial state LOW on all pins
    GPIOD_PDOR &= ~(A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);
}

// -----------------------------------------------------------------------------
// HELPER FUNCTIONS
// -----------------------------------------------------------------------------

void software_delay(unsigned long cycles) {
    while (cycles > 0) {
        __asm("NOP");
        cycles--;
    }
}

void set_step_output(unsigned char output_pattern) {
    // Clear all relevant bits
    GPIOD_PCOR = (A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);

    // Set new pattern
    GPIOD_PSOR = output_pattern;
}

// -----------------------------------------------------------------------------
// MAIN CONTROL LOGIC
// -----------------------------------------------------------------------------

void rotate_stepper_clockwise(void) {
    static unsigned char current_step = 0;

    set_step_output(step_sequence[current_step]);

    // Next step: 0 → 1 → 2 → 3 → 0 ...
    current_step++;
    if (current_step >= 4) {
        current_step = 0;
    }
}

// -----------------------------------------------------------------------------
// MAIN PROGRAM
// -----------------------------------------------------------------------------

int main(void) {
    init_portD_gpio();

    while (1) {
        rotate_stepper_clockwise();
        software_delay(STEP_DELAY_CYCLES);
    }

    // Should never get here
    return 0;
}
