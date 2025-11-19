#include "fsl_device_registers.h"

// Simple test code for stepper motor hardware verification
// This will rotate the motor clockwise with visible speed

// Delay function - adjust count for different speeds
static void delay(volatile unsigned long count)
{
    while (count--)
    {
        __asm("NOP");
    }
}

void main(void)
{
    // 1. Enable clock for PORTD
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    
    // 2. Configure PTD0-PTD5 as GPIO (MUX = 1)
    // Manually configure each pin
    PORTD->PCR[0] = PORT_PCR_MUX(1);  // A1
    PORTD->PCR[1] = PORT_PCR_MUX(1);  // A2
    PORTD->PCR[2] = PORT_PCR_MUX(1);  // B1
    PORTD->PCR[3] = PORT_PCR_MUX(1);  // B2
    PORTD->PCR[4] = PORT_PCR_MUX(1);  // ENA
    PORTD->PCR[5] = PORT_PCR_MUX(1);  // ENB
    
    // 3. Set PTD0-PTD5 as outputs
    GPIOD->PDDR |= 0x3F;  // Bits 0-5
    
    // 4. Initialize all pins LOW
    GPIOD->PDOR = 0x00;
    
    // Give hardware time to stabilize
    delay(1000000);
    
    while (1)
    {
        // CLOCKWISE SEQUENCE (Two-Phase-On Full Step)
        
        // Step 1: A1 + B1 active (both enables ON)
        GPIOD->PDOR = 0x35;  // Binary: 00110101
        delay(2000000);  // Longer delay for visibility
        
        // Step 2: A1 + B2 active
        GPIOD->PDOR = 0x39;  // Binary: 00111001
        delay(2000000);
        
        // Step 3: A2 + B2 active
        GPIOD->PDOR = 0x3A;  // Binary: 00111010
        delay(2000000);
        
        // Step 4: A2 + B1 active
        GPIOD->PDOR = 0x36;  // Binary: 00110110
        delay(2000000);
    }
}
