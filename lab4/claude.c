/*
 * K64F - Duty Cycle Meter with Dual 7-Segment Display
 * Measures pulse width using FTM3 Input Capture
 * Displays duty cycle percentage on two 7-segment displays
 * 
 * Connections:
 * - PWM Input: PTC10 (FTM3_CH6)
 * - 7-Segment Display connections as per your hardware
 */

#include "MK64F12.h"

// 7-segment display digit patterns (common cathode)
const uint8_t SEGMENT_DIGITS[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Assume 7-segment displays connected to Port B (segments) and Port A (digit select)
// Adjust these based on your actual hardware connections
#define SEGMENT_PORT GPIOB
#define DIGIT_PORT GPIOA
#define DIGIT1_PIN 1  // Tens digit
#define DIGIT2_PIN 2  // Units digit

volatile unsigned int t1, t2;
volatile unsigned int pulse_high, pulse_total;
volatile uint8_t capture_state = 0;
volatile uint8_t duty_cycle = 0;

void init_gpio(void);
void init_ftm3_input_capture(void);
void display_digit(uint8_t digit, uint8_t value);
void display_duty_cycle(uint8_t duty);

int main(void) {
    init_gpio();
    init_ftm3_input_capture();
    
    while (1) {
        // Display duty cycle on 7-segment displays
        display_duty_cycle(duty_cycle);
    }
}

void init_gpio(void) {
    // Enable clock for Port A, B, and C
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    
    // Configure PTC10 as FTM3_CH6 (ALT3) for input capture
    PORTC_PCR10 = PORT_PCR_MUX(3);
    
    // Configure Port B pins as GPIO outputs for 7-segment segments
    for (int i = 0; i < 8; i++) {
        PORTB->PCR[i] = PORT_PCR_MUX(1);  // GPIO mode
        GPIOB->PDDR |= (1 << i);           // Output direction
    }
    
    // Configure Port A pins for digit selection
    PORTA->PCR[DIGIT1_PIN] = PORT_PCR_MUX(1);
    PORTA->PCR[DIGIT2_PIN] = PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1 << DIGIT1_PIN) | (1 << DIGIT2_PIN);
}

void init_ftm3_input_capture(void) {
    // Enable FTM3 clock
    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
    
    // Disable FTM3 (write protection)
    FTM3_MODE = 0x05;  // Enable FTM3
    
    // Set modulo to maximum
    FTM3_MOD = 0xFFFF;
    
    // Set prescaler to 32 (System clock / 32)
    FTM3_SC = 0x05;  // Clock source: system clock, prescaler = 32
    
    // Configure Channel 6 for rising edge capture initially
    FTM3_C6SC = 0x44;  // Input capture on rising edge, enable interrupt
    
    // Enable FTM3 Channel 6 interrupt in NVIC
    NVIC_EnableIRQ(FTM3_IRQn);
    
    capture_state = 0;
}

// FTM3 ISR - Measures pulse width and period
void FTM3_IRQHandler(void) {
    if (FTM3_C6SC & 0x80) {  // Check CHF flag
        
        switch (capture_state) {
            case 0:  // First rising edge - start of pulse
                FTM3_CNT = 0;  // Reset counter
                t1 = FTM3_C6V;
                FTM3_C6SC = 0x48;  // Switch to falling edge capture
                capture_state = 1;
                break;
                
            case 1:  // Falling edge - end of high pulse
                t2 = FTM3_C6V;
                pulse_high = t2 - t1;
                FTM3_C6SC = 0x44;  // Switch back to rising edge
                capture_state = 2;
                break;
                
            case 2:  // Second rising edge - end of period
                pulse_total = FTM3_C6V;
                
                // Calculate duty cycle percentage
                if (pulse_total > 0) {
                    duty_cycle = (uint8_t)((pulse_high * 100UL) / pulse_total);
                    if (duty_cycle > 99) duty_cycle = 99;  // Cap at 99%
                }
                
                // Reset for next measurement
                FTM3_CNT = 0;
                t1 = 0;
                FTM3_C6SC = 0x48;  // Capture falling edge next
                capture_state = 1;
                break;
        }
        
        // Clear CHF flag
        FTM3_C6SC &= ~0x80;
    }
}

// Display a single digit on specified 7-segment display
void display_digit(uint8_t digit, uint8_t value) {
    // Turn off all digits first
    GPIOA->PCOR = (1 << DIGIT1_PIN) | (1 << DIGIT2_PIN);
    
    // Set segment pattern
    GPIOB->PDOR = SEGMENT_DIGITS[value];
    
    // Turn on selected digit
    if (digit == 1) {
        GPIOA->PSOR = (1 << DIGIT1_PIN);
    } else if (digit == 2) {
        GPIOA->PSOR = (1 << DIGIT2_PIN);
    }
}

// Display duty cycle percentage with multiplexing
void display_duty_cycle(uint8_t duty) {
    uint8_t tens = duty / 10;
    uint8_t units = duty % 10;
    
    // Multiplex between displays (simple time-based switching)
    for (int i = 0; i < 50; i++) {
        display_digit(1, tens);
        for (volatile int d = 0; d < 1000; d++);  // Delay
        
        display_digit(2, units);
        for (volatile int d = 0; d < 1000; d++);  // Delay
    }
}
