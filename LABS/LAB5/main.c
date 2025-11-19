#include "fsl_device_registers.h"

// Pin Configuration:
// - PORTD PIN 0 (PTD0): A1 output to L298N
//  - PORTD PIN 1 (PTD1): A2 output to L298N
//  - PORTD PIN 2 (PTD2): B1 output to L298N
//  - PORTD PIN 3 (PTD3): B2 output to L298N
//  - PORTB PIN 2 (PTB2): ROT_DIR (0=CW, 1=CCW)
//  - PORTB PIN 3 (PTB3): ROT_SPD (0=22.5deg/s, 1=180deg/s)

#include "MK64F12.h"

// Step patterns for full-step, two-phase on mode
// Each row: {A1, A2, B1, B2}
const uint8_t stepPattern[4][4] = {
    {0, 1, 1, 0},  // Step 0: A1=0, A2=1, B1=1, B2=0
    {1, 0, 1, 0},  // Step 1: A1=1, A2=0, B1=1, B2=0
    {1, 0, 0, 1},  // Step 2: A1=1, A2=0, B1=0, B2=1
    {0, 1, 0, 1}   // Step 3: A1=0, A2=1, B1=0, B2=1
};

// Global variables
volatile int currentStep = 0;
volatile uint8_t direction = 0;  // 0 = CW, 1 = CCW
volatile uint8_t speed = 0;      // 0 = slow (22.5 deg/s), 1 = fast (180 deg/s)

// Function prototypes
void GPIO_Init(void);
void PIT_Init(void);
void readSwitches(void);
void outputStepToMotor(int step);
void advanceStep(void);

int main(void) {
    // Initialize peripherals
    GPIO_Init();
    PIT_Init();
    
    // Main loop
    while(1) {
        readSwitches();  // Continuously read DIP switches
        // Timer interrupt handles stepping
    }
}

/*
 * Initialize GPIO pins
 * PORTD 0-3: Outputs to L298N (motor control)
 * PORTB 2-3: Inputs from DIP switches (with pull-up resistors)
 */
void GPIO_Init(void) {
    // Enable clock to PORTB and PORTD
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    
    // Configure PORTD pins 0-3 as GPIO outputs
    PORTD->PCR[0] = PORT_PCR_MUX(1);  // PTD0 - A1
    PORTD->PCR[1] = PORT_PCR_MUX(1);  // PTD1 - A2
    PORTD->PCR[2] = PORT_PCR_MUX(1);  // PTD2 - B1
    PORTD->PCR[3] = PORT_PCR_MUX(1);  // PTD3 - B2
    
    // Set PORTD pins 0-3 as outputs
    PTD->PDDR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    
    // Initialize all outputs to 0
    PTD->PCOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    
    // Configure PORTB pins 2-3 as GPIO inputs with pull-up resistors
    PORTB->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  // PTB2 - ROT_DIR
    PORTB->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  // PTB3 - ROT_SPD
    
    // Set PORTB pins 2-3 as inputs
    PTB->PDDR &= ~((1 << 2) | (1 << 3));
}

/*
 * Initialize PIT (Periodic Interrupt Timer)
 * Used to generate precise timing for stepper motor steps
 */
void PIT_Init(void) {
    // Enable clock to PIT module
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    
    // Enable PIT module
    PIT->MCR = 0x00;  // Enable PIT, continue in debug mode
    
    // Configure PIT Channel 0
    // Bus clock = 60 MHz (default for K64F)
    // For 22.5 deg/s: 250ms = 250000 us -> 60MHz * 0.25s = 15,000,000 ticks
    PIT->CHANNEL[0].LDVAL = 15000000 - 1;  // Start with slow speed
    
    // Enable interrupts for PIT Channel 0
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK;
    
    // Enable PIT interrupt in NVIC
    NVIC_EnableIRQ(PIT0_IRQn);
    
    // Start timer
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

/*
 * Read DIP switches and update speed/direction
 */
void readSwitches(void) {
    uint8_t newDirection, newSpeed;
    
    // Read ROT_DIR (PTB2) - active low due to pull-up
    // Switch closed (connected to ground) = 0, Switch open = 1
    newDirection = (PTB->PDIR & (1 << 2)) ? 0 : 1;  // 0=CW, 1=CCW
    
    // Read ROT_SPD (PTB3) - active low due to pull-up
    newSpeed = (PTB->PDIR & (1 << 3)) ? 0 : 1;  // 0=slow, 1=fast
    
    // Update global variables
    direction = newDirection;
    
    // Update timer period if speed changed
    if (speed != newSpeed) {
        speed = newSpeed;
        
        // Disable timer while updating
        PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
        
        if (speed == 0) {
            // Slow speed: 22.5 deg/s = 4 steps/sec = 250ms per step
            // 60MHz * 0.25s = 15,000,000 ticks
            PIT->CHANNEL[0].LDVAL = 15000000 - 1;
        } else {
            // Fast speed: 180 deg/s = 32 steps/sec = 31.25ms per step
            // 60MHz * 0.03125s = 1,875,000 ticks
            PIT->CHANNEL[0].LDVAL = 1875000 - 1;
        }
        
        // Re-enable timer
        PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
    }
}

/*
 * Output step pattern to motor
 */
void outputStepToMotor(int step) {
    // Clear all output pins first
    PTD->PCOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    
    // Set pins according to step pattern
    if (stepPattern[step][0]) PTD->PSOR = (1 << 0);  // A1
    if (stepPattern[step][1]) PTD->PSOR = (1 << 1);  // A2
    if (stepPattern[step][2]) PTD->PSOR = (1 << 2);  // B1
    if (stepPattern[step][3]) PTD->PSOR = (1 << 3);  // B2
}

/*
 * Advance to next step based on direction
 */
void advanceStep(void) {
    if (direction == 0) {
        // Clockwise: increment step
        currentStep++;
        if (currentStep > 3) {
            currentStep = 0;
        }
    } else {
        // Counterclockwise: decrement step
        currentStep--;
        if (currentStep < 0) {
            currentStep = 3;
        }
    }
}

/*
 * PIT Channel 0 Interrupt Handler
 * Called at precise intervals to step the motor
 */
void PIT0_IRQHandler(void) {
    // Output current step to motor
    outputStepToMotor(currentStep);
    
    // Advance to next step
    advanceStep();
    
    // Clear interrupt flag
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
}
