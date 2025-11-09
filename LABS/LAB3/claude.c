#include "fsl_device_registers.h"
#include <stdint.h>

// Pin definitions
#define MODE_SW   (1u << 2)   // PTB2: 0 = Counter Mode, 1 = ADC Mode
#define CNT_DIR   (1u << 3)   // PTB3: 0 = Count Up, 1 = Count Down

// Global counter variable
volatile unsigned int cnt = 0;

// Software delay function
void software_delay(unsigned long delay) {
    while (delay > 0) delay--;
}

// Seven-segment patterns for Port D (tens digit)
// Common anode display - segments are active LOW
int numsPTD[10] = {
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011, // 9
};

// Seven-segment patterns for Port C (ones digit)
// Bits: [8,7,6,5,4,3,2,1,0] but bit 7 doesn't exist, bit 6 unused
int numsPTC[10] = {
    0b00111110, // 0
    0b00110000, // 1
    0b01101101, // 2
    0b01111001, // 3
    0b00110011, // 4
    0b01011011, // 5
    0b01011111, // 6
    0b00110000, // 7
    0b01111111, // 8
    0b01111011, // 9
};

// Output digit to Port C (ones place)
void outNumPTC(int num) {
    GPIOC_PDOR = (~numsPTC[num]) & 0x0000003F; // Only use bits 0-5
}

// Output digit to Port D (tens place)
void outNumPTD(int num) {
    GPIOD_PDOR = (~numsPTD[num]) & 0x0000007F; // Only use bits 0-6 (bit 7 is DP)
}

// Display a 2-digit number
void display_2digit(uint8_t val) {
    if (val > 99) val = 99;  // Clamp to 0-99
    uint8_t ones = val % 10;
    uint8_t tens = val / 10;
    outNumPTC(ones);  // Port C = ones digit
    outNumPTD(tens);  // Port D = tens digit
}

// Read 16-bit ADC value
unsigned short ADC_read16b(void) {
    ADC0_SC1A = 0x1A;                              // Start conversion on channel 26
    while (ADC0_SC2 & ADC_SC2_ADACT_MASK);         // Wait for conversion to start
    while (!(ADC0_SC1A & ADC_SC1_COCO_MASK));      // Wait for conversion to complete
    return ADC0_RA;                                 // Return result
}

// Port A Interrupt Handler (triggered by button press on PTA1)
void PORTA_IRQHandler(void) {
    // Clear interrupt flag FIRST
    PORTA_ISFR = (1 << 1);
    
    // Toggle decimal point on PTD7
    GPIOD_PTOR = (1 << 7);
    
    // Check which mode we're in
    if (GPIOB_PDIR & MODE_SW) {
        // ===== ADC MODE =====
        unsigned short adc_val = ADC_read16b();
        // Convert ADC value to display value (0-99)
        // Assuming ADC range 0-65535 maps to 0-99
        unsigned short display_val = (adc_val * 100) / 65536;
        display_2digit(display_val);
        
    } else {
        // ===== COUNTER MODE =====
        if (GPIOB_PDIR & CNT_DIR) {
            // Count DOWN
            if (cnt == 0)
                cnt = 99;
            else
                cnt--;
        } else {
            // Count UP
            if (cnt >= 99)
                cnt = 0;
            else
                cnt++;
        }
        display_2digit(cnt);
    }
}

int main(void) {
    // ===== ENABLE CLOCK GATES =====
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;  // Port A
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Port B
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Port C
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;  // Port D
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;   // ADC0
    
    // ===== CONFIGURE PINS AS GPIO =====
    // Format: (MUX << 16) | pin_mask
    PORTA_GPCLR = (0x0100 << 16) | 0x0002;   // PTA1 as GPIO
    PORTB_GPCLR = (0x0100 << 16) | 0x000C;   // PTB2-3 as GPIO
    PORTC_GPCLR = (0x0100 << 16) | 0x003F;   // PTC0-5 as GPIO (no pin 7!)
    PORTD_GPCLR = (0x0100 << 16) | 0x00FF;   // PTD0-7 as GPIO
    
    // ===== SET PIN DIRECTIONS =====
    GPIOA_PDDR = 0x00000000;    // Port A as Input
    GPIOB_PDDR = 0x00000000;    // Port B as Input
    GPIOC_PDDR = 0x0000003F;    // PTC0-5 as Output
    GPIOD_PDDR = 0x000000FF;    // PTD0-7 as Output (includes DP on bit 7)
    
    // ===== INITIALIZE OUTPUTS =====
    GPIOC_PDOR = 0xFFFFFFFF;    // All high (segments off for common anode)
    GPIOD_PDOR = 0xFFFFFFFF;    // All high (segments off, DP off)
    
    // ===== CONFIGURE ADC =====
    ADC0_CFG1 = 0x0C;           // 16-bit mode, bus clock
    ADC0_SC1A = 0x1F;           // Disable module initially
    
    // ===== CONFIGURE PORT A INTERRUPT =====
    PORTA_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xA);  // GPIO, falling edge interrupt
    PORTA_ISFR = (1 << 1);      // Clear any pending interrupt
    NVIC_EnableIRQ(PORTA_IRQn); // Enable interrupt in NVIC
    
    // ===== INITIALIZE DISPLAY =====
    display_2digit(0);          // Show "00" on startup
    
    // ===== MAIN LOOP =====
    unsigned long Delay = 0x100000;
    while (1) {
        software_delay(Delay);
        // Main loop does nothing - all work done in interrupt handler
    }
    
    return 0;
}
