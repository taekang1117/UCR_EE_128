/*
 * EE128 Lab 5: Stepper Motor Control
 * FRDM-K64F Board
 * Controls a bipolar stepper motor using L298N driver
 * 
 * Pin Configuration:
 * - Motor Control Outputs: Port D (PD0-PD5)
 *   - PD0: IN1/A1 (Yellow wire)
 *   - PD1: IN2/A2 (White wire)
 *   - PD2: IN3/B1 (Red wire)
 *   - PD3: IN4/B2 (Blue wire)
 *   - PD4: ENA (Enable for Motor A)
 *   - PD5: ENB (Enable for Motor B)
 * - DIP Switch Inputs: Port B (PB2-PB3)
 *   - PB2: ROT_DIR (0=CW, 1=CCW)
 *   - PB3: ROT_SPD (0=22.5°/s, 1=180°/s)
 */

#include "fsl_device_registers.h"

// Step patterns for two-phase on, full-step mode
// Format: {A1, A2, B1, B2}
int stepPattern[4][4] = {
    {0, 1, 1, 0},  // Step 0
    {1, 0, 1, 0},  // Step 1
    {1, 0, 0, 1},  // Step 2
    {0, 1, 0, 1}   // Step 3
};

volatile int currentStep = 0;
volatile uint8_t direction = 0;  // 0 = CW, 1 = CCW
volatile uint8_t speed = 0;      // 0 = slow, 1 = fast

void init_gpio(void);
void init_pit(void);
void read_switches(void);
void output_step(int step);

int main(void) {
    init_gpio();
    init_pit();
    
    while (1) {
        // Continuously read DIP switches for mode changes
        read_switches();
    }
}

void init_gpio(void) {
    // Enable clock for Port B and D
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    
    // Configure Port D pins 0-3 as GPIO (motor control outputs)
    PORTD_PCR0 = 0x100;  // IN1/A1
    PORTD_PCR1 = 0x100;  // IN2/A2
    PORTD_PCR2 = 0x100;  // IN3/B1
    PORTD_PCR3 = 0x100;  // IN4/B2
    
    // Configure Port D pins 4-5 as GPIO (L298N enable pins)
    PORTD_PCR4 = 0x100;  // ENA
    PORTD_PCR5 = 0x100;  // ENB
    
    // Configure Port B pins 2-3 as GPIO with pull-up (DIP switch inputs)
    PORTB_PCR2 = 0x103;  // GPIO with pull-up for ROT_DIR
    PORTB_PCR3 = 0x103;  // GPIO with pull-up for ROT_SPD
    
    // Set Port D pins 0-5 as outputs
    GPIOD_PDDR |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5);
    
    // Set Port B pins 2-3 as inputs
    GPIOB_PDDR &= ~((1<<2) | (1<<3));
    
    // Initialize all motor control outputs to LOW
    GPIOD_PDOR &= ~0x0F;
    
    // Enable the L298N driver by setting ENA and ENB HIGH
    GPIOD_PDOR |= (1<<4) | (1<<5);  // Set PD4 (ENA) and PD5 (ENB) HIGH
}

void init_pit(void) {
    // Enable PIT clock
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    
    // Enable PIT module (MCR = 0x00)
    PIT_MCR = 0x00;
    
    // Configure PIT Timer 0
    // Bus clock = 60 MHz
    // Slow speed: 22.5°/s = 4 steps/sec = 250ms
    // 60 MHz * 0.25s = 15,000,000 ticks
    PIT_LDVAL0 = 15000000 - 1;
    
    // Enable Timer 0 interrupts
    PIT_TCTRL0 = 0x02;  // TIE = 1 (interrupt enabled)
    
    // Clear any pending PIT0 interrupt flags
    PIT_TFLG0 = 0x01;
    
    // Clear any pending interrupts in NVIC
    NVIC_ClearPendingIRQ(PIT0_IRQn);
    
    // Enable PIT interrupt in NVIC
    NVIC_EnableIRQ(PIT0_IRQn);
    
    // Start Timer 0
    PIT_TCTRL0 |= 0x01;  // TEN = 1 (timer enabled)
}

void read_switches(void) {
    uint8_t newDirection, newSpeed;
    
    // Read DIP switches (active low with pull-up)
    // Switch closed (grounded) = 0, open = 1
    newDirection = (GPIOB_PDIR & (1<<2)) ? 0 : 1;  // PB2: ROT_DIR
    newSpeed = (GPIOB_PDIR & (1<<3)) ? 0 : 1;      // PB3: ROT_SPD
    
    // Update direction
    direction = newDirection;
    
    // Update speed and timer period if changed
    if (speed != newSpeed) {
        speed = newSpeed;
        
        // Disable timer while updating load value
        PIT_TCTRL0 &= ~0x01;
        
        if (speed == 0) {
            // Slow: 22.5°/s = 250ms per step
            PIT_LDVAL0 = 15000000 - 1;
        } else {
            // Fast: 180°/s = 31.25ms per step
            PIT_LDVAL0 = 1875000 - 1;
        }
        
        // Re-enable timer
        PIT_TCTRL0 |= 0x01;
    }
}

void output_step(int step) {
    uint32_t output = 0;
    
    // Build output word from step pattern
    if (stepPattern[step][0]) output |= (1<<0);  // A1
    if (stepPattern[step][1]) output |= (1<<1);  // A2
    if (stepPattern[step][2]) output |= (1<<2);  // B1
    if (stepPattern[step][3]) output |= (1<<3);  // B2
    
    // Output to Port D (clear bits 0-3, then set according to pattern)
    GPIOD_PDOR = (GPIOD_PDOR & ~0x0F) | (output & 0x0F);
}

// PIT Timer 0 ISR - Steps the motor at precise intervals
void PIT0_IRQHandler(void) {
    // Output current step pattern to motor
    output_step(currentStep);
    
    // Advance to next step based on direction
    if (direction == 0) {
        // Clockwise: increment
        currentStep++;
        if (currentStep > 3) {
            currentStep = 0;
        }
    } else {
        // Counterclockwise: decrement
        currentStep--;
        if (currentStep < 0) {
            currentStep = 3;
        }
    }
    
    // Clear Timer 0 interrupt flag
    PIT_TFLG0 = 0x01;
}
