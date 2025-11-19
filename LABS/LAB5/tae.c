#include "fsl_device_registers.h"

// -----------------------------------------------------------------------------
// PIN DEFINITIONS (PORT D)
// -----------------------------------------------------------------------------

// INITIALIZATION
// Motor Phase A (Connected to L298 OUT1 & OUT2)
#define A1_PIN (1u << 0) // PORT D, Pin 0 
#define A2_PIN (1u << 1) // PORT D, Pin 1

// Motor Phase B (Connected to L298 OUT3 & OUT4)
#define B1_PIN (1u << 2) // PORT D, Pin 2
#define B2_PIN (1u << 3) // PORT D, Pin 3

// L298 Enable Pins
#define ENA_PIN (1u << 4) // PORT D, Pin 4 (Enables Phase A outputs OUT1/OUT2)
#define ENB_PIN (1u << 5) // PORT D, Pin 5 (Enables Phase B outputs OUT3/OUT4)

// Stepper control sequence (One-Phase On, Clockwise)
const unsigned char step_sequence[4] = {
    (A1_PIN | ENA_PIN),             // Step 1: Phase A forward
    (B1_PIN | ENB_PIN),             // Step 2: Phase B forward
    (A2_PIN | ENA_PIN),             // Step 3: Phase A reverse
    (B2_PIN | ENB_PIN)              // Step 4: Phase B reverse
};

// Delay calibration for ~80ms step rate
#define STEP_DELAY_CYCLES 2500000

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
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // 2. Configure PTD0-PTD5 as GPIO (Mux value 1)
    PORTD->GPCLR = (0x0100 << 16) | (A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);

    // 3. Set PTD0-PTD5 as Outputs
    GPIOD->PDDR |= (A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);
    
    // 4. Set Initial State to OFF (all outputs low)
    GPIOD->PDOR = 0; 
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
    // 1. Disable all outputs first
    GPIOD->PCOR = (A1_PIN | A2_PIN | B1_PIN | B2_PIN | ENA_PIN | ENB_PIN);
    
    // 2. Apply the new pattern
    GPIOD->PSOR = output_pattern;
}

// -----------------------------------------------------------------------------
// MAIN CONTROL LOGIC
// -----------------------------------------------------------------------------

void rotate_stepper_clockwise(void) {
    static unsigned char current_step = 0;
    
    // Rotate in the forward direction (Clockwise)
    set_step_output(step_sequence[current_step]);

    // Move to the next step (0 -> 1 -> 2 -> 3 -> 0...)
    current_step++;
    if (current_step >= 4) {
        current_step = 0;
    }
}

// -----------------------------------------------------------------------------
// MAIN PROGRAM
// -----------------------------------------------------------------------------

int main(void) {
    // Initialization
    init_portD_gpio();

    while (1) {
        // Perform one step
        rotate_stepper_clockwise();

        // Wait for the calculated delay (80ms)
        software_delay(STEP_DELAY_CYCLES);
    }
    return 0;
}
