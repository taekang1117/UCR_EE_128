#include "fsl_device_registers.h"

// Step patterns for two-phase on, full-step mode
// Format per row: {A1, A2, B1, B2}
const uint8_t stepPattern[4][4] = {
    {0, 1, 1, 0},  // Step 0
    {1, 0, 1, 0},  // Step 1
    {1, 0, 0, 1},  // Step 2
    {0, 1, 0, 1}   // Step 3
};

volatile int      currentStep = 0;
volatile uint8_t direction   = 0;  // 0 = CW, 1 = CCW
volatile uint8_t speed       = 0;  // 0 = slow, 1 = fast

void init_gpio(void);
void init_pit(void);
void read_switches(void);
void output_step(int step);

int main(void) {
    init_gpio();
    init_pit();

    // If your startup code doesn’t already do this, you may need:
    // __enable_irq();

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
    PORTD_PCR0 = 0x100;  // MUX = 001 (GPIO) for IN1/A1
    PORTD_PCR1 = 0x100;  // IN2/A2
    PORTD_PCR2 = 0x100;  // IN3/B1
    PORTD_PCR3 = 0x100;  // IN4/B2
    
    // Configure Port D pins 4-5 as GPIO (L298N enable pins)
    PORTD_PCR4 = 0x100;  // ENA
    PORTD_PCR5 = 0x100;  // ENB
    
    // Configure Port B pins 2-3 as GPIO with pull-up (DIP switch inputs)
    // 0x103 = MUX(001) + PE(1) + PS(1) → GPIO + pull-up resistor, active-low switch
    PORTB_PCR2 = 0x103;  // ROT_DIR
    PORTB_PCR3 = 0x103;  // ROT_SPD
    
    // Set Port D pins 0-5 as outputs
    GPIOD_PDDR |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5);
    
    // Set Port B pins 2-3 as inputs
    GPIOB_PDDR &= ~((1<<2) | (1<<3));
    
    // Initialize all motor control outputs (PD0–PD5) to LOW
    GPIOD_PDOR &= ~0x3F;          // clear bits 0..5
    
    // Enable the L298N driver by setting ENA and ENB HIGH
    GPIOD_PDOR |= (1<<4) | (1<<5);  // PD4 (ENA) and PD5 (ENB) HIGH
}

void init_pit(void) {
    // Enable PIT clock
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    
    // Enable PIT module (MDIS = 0, FRZ = 0)
    PIT_MCR = 0x00;
    
    // Configure PIT Timer 0
    // Bus clock = 60 MHz
    // Slow:   4 steps/sec  → 0.25 s/step → 60e6 * 0.25 = 15,000,000 ticks
    PIT_LDVAL0 = 15000000 - 1;
    
    // Enable Timer 0 interrupts (TIE = 1, TEN initially 0 here)
    PIT_TCTRL0 = 0x02;  
    
    // Clear any pending PIT0 interrupt flags
    PIT_TFLG0 = 0x01;
    
    // Clear any pending interrupts in NVIC and enable PIT0 interrupt
    NVIC_ClearPendingIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);
    
    // Start Timer 0 (TEN = 1)
    PIT_TCTRL0 |= 0x01;
}

void read_switches(void) {
    uint8_t newDirection, newSpeed;
    
    // Read DIP switches (active-low)
    // Switch closed = 0 (grounded), open = 1 (pulled up)
    newDirection = (GPIOB_PDIR & (1<<2)) ? 0 : 1;  // PB2: ROT_DIR
    newSpeed     = (GPIOB_PDIR & (1<<3)) ? 0 : 1;  // PB3: ROT_SPD
    
    // Update direction (atomic write to 8-bit variable)
    direction = newDirection;
    
    // Update speed and timer period only if it changed
    if (speed != newSpeed) {
        speed = newSpeed;
        
        // Disable timer while updating load value (TEN = 0)
        PIT_TCTRL0 &= ~0x01;
        
        if (speed == 0) {
            // Slow: 22.5°/s = 4 steps/sec = 250 ms per step
            PIT_LDVAL0 = 15000000 - 1;
        } else {
            // Fast: 180°/s = 32 steps/sec ≈ 31.25 ms per step
            // 60e6 * 0.03125 = 1,875,000
            PIT_LDVAL0 = 1875000 - 1;
        }
        
        // Re-enable timer (TEN = 1)
        PIT_TCTRL0 |= 0x01;
    }
}

void output_step(int step) {
    uint32_t output = 0;
    
    // Build bit pattern from step table:
    // stepPattern[step][0] → A1 → PD0
    // stepPattern[step][1] → A2 → PD1
    // stepPattern[step][2] → B1 → PD2
    // stepPattern[step][3] → B2 → PD3
    if (stepPattern[step][0]) output |= (1<<0);  // A1
    if (stepPattern[step][1]) output |= (1<<1);  // A2
    if (stepPattern[step][2]) output |= (1<<2);  // B1
    if (stepPattern[step][3]) output |= (1<<3);  // B2
    
    // Clear bits 0–3 (coil pins), then OR in new pattern
    GPIOD_PDOR = (GPIOD_PDOR & ~0x0F) | (output & 0x0F);
}

// PIT Timer 0 ISR - steps the motor at precise intervals
void PIT0_IRQHandler(void) {
    // Output current step pattern
    output_step(currentStep);
    
    // Advance to next step based on direction
    if (direction == 0) {
        // Clockwise: 0 → 1 → 2 → 3 → 0 ...
        currentStep++;
        if (currentStep > 3) {
            currentStep = 0;
        }
    } else {
        // Counterclockwise: 3 → 2 → 1 → 0 → 3 ...
        currentStep--;
        if (currentStep < 0) {
            currentStep = 3;
        }
    }
    
    // Clear Timer 0 interrupt flag (write 1 to TIF)
    PIT_TFLG0 = 0x01;
}
